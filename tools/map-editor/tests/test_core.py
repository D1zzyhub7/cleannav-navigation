import json
import math
from html.parser import HTMLParser
from pathlib import Path
import re
import sys
import threading
import tempfile
import shutil
import unittest
from urllib.request import Request, urlopen

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
from core import DEFAULT_MAP, DEMO_MAP, MapModel, check_planner_path, export_registry, plan
from server import App, Handler
from http.server import ThreadingHTTPServer
from shapely.geometry import Point
from map_codec import encode_trinary
import numpy as np
from PIL import Image
from core import digest
from unittest.mock import patch
from fixture_helpers import load_site, unpack_projection
import server




class PerimeterPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.demo = MapModel(DEMO_MAP)
        cls.campus = load_site('campus')
        cls.scene_temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.scene_temp.cleanup)
        cls.scene = unpack_projection(Path(cls.scene_temp.name) / 'scene')
        cls.boundary = [[1, 1], [14, 1], [14, 14], [1, 14]]

    def test_demo_map_is_explicit_synthetic_fixture(self):
        self.assertTrue(self.demo.info()['test_only'])
        self.assertTrue(self.demo.info()['identity']['map_id'].startswith('DEMO-'))
        self.assertTrue(self.campus.info()['test_only'])  # derived real-map fixture, never runtime
        self.assertEqual(self.campus.info()['identity']['map_id'], 'campus_curb_scan_20261012_083555_fd1b1b')
        self.assertEqual(len(self.campus.profile_sha256), 64)

    def test_coordinate_conversion_round_trip_with_rotated_origin(self):
        model = MapModel.__new__(MapModel)
        model.resolution, model.origin, model.width, model.height = .05, [-2.3, 4.2, .37], 300, 240
        point = [3.471, -1.117]
        u, v = model.map_to_pixel(*point)
        recovered = model.pixel_to_map(u, v)
        self.assertAlmostEqual(point[0], recovered[0], places=10)
        self.assertAlmostEqual(point[1], recovered[1], places=10)

    def test_fixture_generates_closed_radius_bounded_safe_loop(self):
        result = plan(self.demo, self.boundary, margin=.15, direction='ccw', laps=3)
        self.assertTrue(result['feasible'], result.get('reason'))
        report = result['report']
        self.assertFalse(report['motion_validated'])
        self.assertFalse(report['planner_segments_validated'])
        self.assertEqual(report['checkpoint_count'], 9)
        self.assertEqual(report['sequence_goals_with_goto_return'], 27)
        self.assertAlmostEqual(result['waypoints'][0][0], result['waypoints'][-1][0])
        self.assertAlmostEqual(result['waypoints'][0][1], result['waypoints'][-1][1])
        self.assertLessEqual(report['max_sample_curvature'], 1/report['minimum_radius_m']*1.001)
        self.assertGreaterEqual(report['footprint_disk_radius_m'], .64)

    def test_clockwise_and_counterclockwise_are_generated(self):
        ccw = plan(self.demo, self.boundary, direction='ccw')
        cw = plan(self.demo, self.boundary, direction='cw')
        self.assertTrue(ccw['feasible'], ccw.get('reason'))
        self.assertTrue(cw['feasible'], cw.get('reason'))
        self.assertNotEqual(ccw['waypoints'][0][2], cw['waypoints'][0][2])

    def test_narrow_region_and_bad_polygon_are_rejected(self):
        narrow = plan(self.demo, [[1, 1], [3, 1], [3, 8], [1, 8]])
        self.assertFalse(narrow['feasible'])
        with self.assertRaises(ValueError):
            plan(self.demo, [[1, 1], [4, 4], [1, 4], [4, 1]])

    def test_campus_does_not_claim_a_perimeter_loop_for_test_area(self):
        result = plan(self.campus, [[1, -2], [8, -2], [8, 5], [1, 5]], laps=3)
        self.assertFalse(result['feasible'])
        self.assertIn('No checked loop found', result['reason'])

    def test_gazebo_world_projection_binds_world_map_and_mission_regions(self):
        # Preserve historical map bytes; reproduce and repair the old 205/.25
        # export in a private test bundle before testing the complete editor.
        with self.assertRaisesRegex(ValueError, 'unknown cells'):
            MapModel(self.scene)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        directory = Path(temporary.name) / 'projection'
        shutil.copytree(self.scene, directory)
        grid = json.loads((directory / 'occupancy_grid.json').read_text())
        Image.fromarray(encode_trinary(np.array(grid['data']).reshape(grid['height'], grid['width']))).save(directory / 'map.pgm')
        manifest = json.loads((directory / 'manifest.json').read_text())
        manifest['source_world'] = str((self.scene / manifest['source_world']).resolve())
        manifest['sha256']['map.pgm'] = digest(directory / 'map.pgm')
        (directory / 'manifest.json').write_text(json.dumps(manifest))
        model = MapModel(directory)
        self.assertFalse(model.info()['test_only'])
        self.assertIn('not RTAB sensor map', model.info()['provenance'])
        self.assertEqual(model.identity['map_id'], 'gazebo-patrol-v1')
        regions = [
            ('A', [[-15.6,-2.95],[-5.55,-2.95],[-5.55,7.2],[-15.6,7.2]]),
            ('B', [[-4.7,-2.95],[4.7,-2.95],[4.7,7.2],[-4.7,7.2]]),
            ('C', [[5.55,-2.95],[15.6,-2.95],[15.6,7.2],[5.55,7.2]]),
        ]
        for _, boundary in regions:
            result = plan(model, boundary, margin=.15, direction='ccw', laps=3)
            self.assertTrue(result['feasible'], result.get('reason'))
            self.assertEqual(result['report']['checkpoint_count'], 9)
            self.assertEqual(result['report']['sequence_goals_with_goto_return'], 27)
        charger = model.free.covers(Point(0.0, -8.4).buffer(model.vehicle['disk_radius']+.15+.005))
        self.assertTrue(charger)

    def test_smac_curvature_uses_geometry_not_quantized_heading_delta(self):
        radius, centre = 1.12, (7.5, 7.5)
        poses = []
        for i in range(12):
            angle = i*.07
            yaw = angle+math.pi/2
            yaw = round(yaw/(math.pi/36))*(math.pi/36)  # Smac's 5-degree heading bins
            poses.append([centre[0]+radius*math.cos(angle),
                          centre[1]+radius*math.sin(angle), yaw])
        result = check_planner_path(self.demo, self.boundary, poses, poses, corridor=.05,
                                    require_forward=True)
        self.assertTrue(result['turning_radius_pass'], result)
        self.assertTrue(result['forward_motion_pass'], result)
        self.assertLess(result['max_sample_curvature'], .90)

    def test_registry_is_json_schema_and_mission_loader_compatible(self):
        project = dict(map=self.demo.identity, chargers=[dict(charger_id='demo_charger', display_name='演示充电点',
                      staging_pose=dict(x=3.0, y=3.0, yaw=0.0))],
                      regions=[dict(region_id='A', display_name='演示A区', route_id='perimeter',
                                   boundary=self.boundary, options=dict(margin=.15, direction='ccw', laps=3))])
        output = export_registry(self.demo, project)
        registry = output['registry']
        self.assertEqual(set(registry), {'schema_version', 'map', 'regions', 'chargers'})
        self.assertEqual(registry['regions'][0]['entry_pose'], registry['regions'][0]['routes'][0]['waypoints'][0])
        self.assertEqual(output['validation'][0]['checkpoint_count'], 9)
        self.assertEqual(output['status'], 'OFFLINE_GEOMETRY_ONLY_NOT_MOTION_ACCEPTED')
        wrong = dict(project)
        wrong['map'] = dict(self.demo.identity, sha256='0'*64)
        with self.assertRaises(ValueError):
            export_registry(self.demo, wrong)
        unsafe = dict(project)
        unsafe['chargers'] = [dict(charger_id='blocked', staging_pose=dict(x=7.5, y=7.5, yaw=0.0))]
        with self.assertRaises(ValueError):
            export_registry(self.demo, unsafe)

    def test_saved_smac_path_checks_collision_region_corridor_and_curvature(self):
        candidate = plan(self.demo, self.boundary)
        ref = candidate['dense']
        safe = [[5.0, 5.0, 0.0], [6.0, 5.0, 0.0], [6.5, 5.0, 0.0]]
        # This line is intentionally far from the perimeter route, so it must
        # not pass merely because it is collision-free.
        result = check_planner_path(self.demo, self.boundary, ref, safe)
        self.assertTrue(result['obstacle_clear'])
        self.assertTrue(result['stays_in_region'])
        self.assertFalse(result['stays_in_route_corridor'])
        blocked = [[6.0, 7.5, 0.0], [9.0, 7.5, 0.0]]
        result = check_planner_path(self.demo, self.boundary, ref, blocked)
        self.assertFalse(result['obstacle_clear'])
        unknown = [[3.5, 10.25, 0.0], [5.0, 10.25, 0.0]]
        result = check_planner_path(self.demo, self.boundary, ref, unknown)
        self.assertFalse(result['obstacle_clear'])
        with self.assertRaises(ValueError):
            check_planner_path(self.demo, self.boundary, ref, [])


class LocalEditorApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene_temp = tempfile.TemporaryDirectory()
        cls.scene = unpack_projection(Path(cls.scene_temp.name) / 'scene')
        cls.maps = patch.dict(server.MAPS, {'saved-gazebo-patrol-v1': cls.scene})
        cls.maps.start()
        cls.http = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        cls.http.app = App('demo-fixture')
        cls.thread = threading.Thread(target=cls.http.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f'http://127.0.0.1:{cls.http.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown(); cls.http.server_close(); cls.thread.join(timeout=2)
        cls.maps.stop(); cls.scene_temp.cleanup()

    def test_editor_static_app_map_and_plan_api(self):
        with urlopen(self.base+'/') as response:
            page = response.read().decode()
        self.assertIn('自动生成安全边缘环线', page)
        with urlopen(self.base+'/app.js') as response:
            app_js = response.read().decode()
        self.assertIn("$('exportBtn').onclick", app_js)
        with urlopen(self.base+'/style.css') as response:
            self.assertIn('map-wrap', response.read().decode())
        with urlopen(self.base+'/api/maps') as response:
            maps = json.loads(response.read())
        self.assertIn('demo-fixture', [item['key'] for item in maps['maps']])
        self.assertIn('saved-gazebo-patrol-v1', [item['key'] for item in maps['maps']])
        with urlopen(self.base+'/api/info') as response:
            info = json.loads(response.read())
        self.assertTrue(info['test_only'])
        with urlopen(self.base+'/api/map.png') as response:
            self.assertEqual(response.headers.get_content_type(), 'image/png')
        data = json.dumps(dict(boundary=[[1,1],[14,1],[14,14],[1,14]], options=dict(laps=3))).encode()
        req = Request(self.base+'/api/plan', data=data, headers={'Content-Type':'application/json'})
        with urlopen(req) as response:
            result = json.loads(response.read())
        self.assertTrue(result['feasible'], result.get('reason'))
        project = dict(map=self.http.app.model.identity,
                       regions=[dict(region_id='A',display_name='演示A区',route_id='perimeter',
                                     boundary=[[1,1],[14,1],[14,14],[1,14]],
                                     options=dict(margin=.15,direction='ccw',laps=3))],
                       chargers=[dict(charger_id='demo_charger',display_name='演示充电点',
                                      staging_pose=dict(x=3.0,y=3.0,yaw=0.0))])
        data = json.dumps(project).encode()
        req = Request(self.base+'/api/export', data=data, headers={'Content-Type':'application/json'})
        with urlopen(req) as response:
            exported = json.loads(response.read())
        self.assertEqual(exported['registry']['map'], project['map'])
        self.assertEqual(exported['registry']['regions'][0]['region_id'], 'A')

    def test_html_controls_match_javascript_dom_bindings_and_start_after_dom_ready(self):
        class IdParser(HTMLParser):
            def __init__(self):
                super().__init__()
                self.ids = set()
            def handle_starttag(self, tag, attrs):
                value = dict(attrs).get('id')
                if value:
                    self.ids.add(value)

        with urlopen(self.base+'/') as response:
            page = response.read().decode()
        with urlopen(self.base+'/app.js') as response:
            script = response.read().decode()
        parser = IdParser()
        parser.feed(page)
        direct_bindings = set(re.findall(r"\$\('([^']+)'\)", script))
        required_match = re.search(r'const requiredIds = \[(.*?)\];', script, re.S)
        self.assertIsNotNone(required_match, 'required DOM control validation is missing')
        required_ids = set(re.findall(r"'([^']+)'", required_match.group(1)))
        self.assertTrue(direct_bindings <= parser.ids, sorted(direct_bindings-parser.ids))
        self.assertTrue(required_ids <= parser.ids, sorted(required_ids-parser.ids))
        self.assertTrue(direct_bindings <= required_ids, sorted(direct_bindings-required_ids))
        self.assertIn("yaw:'chargerYaw'", script)
        self.assertNotIn('charger${field.toUpperCase()}', script)
        self.assertIn("document.addEventListener('DOMContentLoaded', initializeMapEditor, {once:true})", script)
        self.assertIn("init();", script)
        self.assertIn("['A','B','C']", script)
        self.assertIn("if(drawMode==='boundary'){draft.push(world);draw();return;}", script)
        self.assertIn('function finishBoundary()', script)
        self.assertIn("$('planBtn').onclick", script)
        self.assertIn("$('exportBtn').onclick", script)


if __name__ == '__main__':
    unittest.main(verbosity=2)
