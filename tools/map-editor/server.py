#!/usr/bin/env python3
"""Local-only HTTP API and static host for the CleanNav map editor."""
from __future__ import annotations
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from core import DEFAULT_MAP, DEMO_MAP, ROOT, MapModel, check_planner_path, export_registry, plan, validate_registry

STATIC = Path(__file__).resolve().parent/'static'
MAPS = {'campus': DEFAULT_MAP, 'demo-fixture': DEMO_MAP}


def discover_saved_maps():
    """Expose saved map bundles under experiment maps/ directories by identity."""
    patterns = ('*/maps/*/manifest.json', '*/maps/manifest.json',
                '*/scene/maps/*/manifest.json')
    seen = {Path(DEFAULT_MAP).resolve()}
    for pattern in patterns:
        for manifest_path in sorted((ROOT/'cleannav-experiments').glob(pattern)):
            directory = manifest_path.parent.resolve()
            if directory in seen or not (directory/'occupancy_grid.json').is_file():
                continue
            try:
                manifest = json.loads(manifest_path.read_text())
                map_id = manifest.get('name')
                if not isinstance(map_id, str) or not map_id:
                    continue
                seen.add(directory)
                key = 'saved-'+map_id
                if key not in MAPS:
                    MAPS[key] = directory
            except (OSError, json.JSONDecodeError):
                continue


discover_saved_maps()


def public_map_list():
    out = []
    for key, path in MAPS.items():
        try:
            model = MapModel(path)
            info = model.info()
            profile = ' · formal Keepout bound' if info['formal_keepout_loaded'] else ' · no separate Keepout profile'
            out.append(dict(key=key, label=f"{info['map_label']} · {info['identity']['map_id']}"+profile,
                            map_id=info['identity']['map_id'], test_only=info['test_only']))
        except Exception as exc:
            out.append(dict(key=key, label=key, unavailable=str(exc)))
    return out


class App:
    def __init__(self, map_key):
        self.map_key = map_key
        self.model = MapModel(MAPS[map_key])


class Handler(BaseHTTPRequestHandler):
    server_version = 'CleanNavMapEditor/1.0'

    def _send(self, code, body, content_type='application/json; charset=utf-8'):
        if isinstance(body, str):
            payload = body.encode()
        else:
            payload = body
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(payload)

    def _json(self, code, data):
        self._send(code, json.dumps(data, ensure_ascii=False, allow_nan=False), 'application/json; charset=utf-8')

    def _body(self):
        length = int(self.headers.get('Content-Length', '0'))
        if length < 1 or length > 2_000_000:
            raise ValueError('Request body is empty or exceeds 2 MB')
        raw = self.rfile.read(length)
        try:
            return json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f'Invalid JSON: {exc}') from exc

    def _guard_local(self):
        if self.client_address[0] not in ('127.0.0.1', '::1'):
            self._json(403, {'error': 'This editor only accepts local connections'})
            return False
        if self.headers.get('Origin') not in (None, 'null', f'http://127.0.0.1:{self.server.server_port}',
                                               f'http://localhost:{self.server.server_port}'):
            self._json(403, {'error': 'Cross-origin requests are disabled'})
            return False
        return True

    def do_GET(self):
        path = urlparse(self.path).path
        if path.startswith('/api/') and not self._guard_local():
            return
        app = self.server.app
        try:
            if path == '/api/maps':
                return self._json(200, {'maps': public_map_list(), 'selected': app.map_key})
            if path == '/api/info':
                return self._json(200, app.model.info())
            if path == '/api/map.png':
                return self._send(200, app.model.image(), 'image/png')
            if path == '/':
                return self._send(200, (STATIC/'index.html').read_text(), 'text/html; charset=utf-8')
            if path in ('/app.js', '/style.css'):
                asset = STATIC/path.removeprefix('/')
                kind = 'text/javascript; charset=utf-8' if path.endswith('.js') else 'text/css; charset=utf-8'
                return self._send(200, asset.read_text(), kind)
            return self._json(404, {'error': 'Not found'})
        except Exception as exc:
            return self._json(500, {'error': str(exc)})

    def do_POST(self):
        if not self._guard_local():
            return
        path = urlparse(self.path).path
        try:
            body = self._body()
            if path == '/api/load-map':
                key = body.get('key')
                if key not in MAPS:
                    raise ValueError('Unknown map key')
                candidate = MapModel(MAPS[key])
                self.server.app.model, self.server.app.map_key = candidate, key
                return self._json(200, candidate.info())
            if path == '/api/plan':
                result = plan(self.server.app.model, body.get('boundary'), **body.get('options', {}))
                return self._json(200, result)
            if path == '/api/export':
                result = export_registry(self.server.app.model, body)
                return self._json(200, result)
            if path == '/api/validate-registry':
                return self._json(200, {'valid': True,
                                        'registry': validate_registry(self.server.app.model, body),
                                        'loaded_by': 'Mission RegionRegistry.load + require_map'})
            if path == '/api/check-planner-path':
                result = check_planner_path(self.server.app.model, body['boundary'],
                                            body['candidate_dense'], body['actual_poses'],
                                            body.get('corridor', .25))
                return self._json(200, result)
            return self._json(404, {'error': 'Not found'})
        except Exception as exc:
            return self._json(400, {'error': str(exc)})

    def log_message(self, fmt, *args):
        print('%s - %s' % (self.address_string(), fmt % args), flush=True)


def main():
    parser = argparse.ArgumentParser(description='Local CleanNav map editor; offline only.')
    parser.add_argument('--map', choices=MAPS, default='campus')
    parser.add_argument('--host', default='127.0.0.1', help='Loopback only; remote binding is rejected')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    if args.host not in ('127.0.0.1', '::1'):
        parser.error('Only loopback binding is supported')
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.daemon_threads = True
    httpd.app = App(args.map)
    print(f'CleanNav map editor on http://{args.host}:{httpd.server_port}/ (map={args.map})', flush=True)
    print('Offline only: no ROS clients, Mission targets, cmd_vel, Gazebo or robot runtime.', flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\nEditor stopped.', flush=True)
    finally:
        httpd.server_close()


if __name__ == '__main__':
    main()
