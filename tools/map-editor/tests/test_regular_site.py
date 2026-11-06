"""R9 real Pillar fixture, fixed production start and forward direction only."""
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import unittest
import numpy as np
from PIL import Image
from shapely.geometry import LineString, Polygon

TOOL = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(TOOL), str(Path(__file__).resolve().parent)]
from core import plan, export_registry, _swept_oriented_footprint
from map_codec import decode_trinary
from fixture_helpers import load_site, captured

REPO = TOOL.parents[1]
spec = importlib.util.spec_from_file_location('source_a4', REPO / 'cleannav_navigation/patrol_path_segments.py')
a4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a4)


def geometric_curvature(poses):
    xy = np.asarray(poses)[:-1, :2]
    a, b = xy-np.roll(xy, 1, axis=0), np.roll(xy, -1, axis=0)-xy
    den = np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)*np.linalg.norm(a+b, axis=1)
    return np.divide(2*(a[:,0]*b[:,1]-a[:,1]*b[:,0]), den, out=np.zeros(len(xy)), where=den>1e-12)


class RegularSiteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = load_site('pillar')
        cls.reference = json.loads(captured('pillar/C-route.json.gz'))
        cls.boundary = cls.reference['cruise_boundary']
        cls.result = plan(cls.model, cls.boundary, margin=.15, direction='ccw', laps=1)
        assert cls.result['feasible'], cls.result
        cls.poses = cls.result['dense']

    def test_actual_source_reproduces_C_geometry(self):
        expected = [[p['x'],p['y'],p['yaw']] for p in self.reference['dense_path_map']]
        self.assertEqual(len(self.poses), 1637)
        np.testing.assert_allclose(self.poses, expected, atol=1e-12, rtol=0)
        self.assertEqual(self.model.vehicle['turning_radius'], 1.12)
        self.assertAlmostEqual(self.result['report']['minimum_radius_m'], 1.344)

    def test_four_corners_radius_tangency_and_curvature_distribution(self):
        k = geometric_curvature(self.poses)
        self.assertLessEqual(max(abs(k)), .74405)
        # Four separated nonzero-curvature runs; each circle has R=1.344.
        active = abs(k)>1e-6
        starts = [i for i in range(len(k)) if active[i] and not active[i-1]]
        self.assertEqual(len(starts), 4)
        for start in starts:
            run=[];i=start
            while active[i % len(k)]:
                run.append(abs(k[i % len(k)]));i+=1
            self.assertAlmostEqual(1/max(run), 1.344, places=8)
        wp=self.result['waypoints']
        for i in range(4):
            a,b=wp[2*i+1],wp[(2*i+2)%8]
            heading=math.atan2(b[1]-a[1],b[0]-a[0])
            for endpoint in (a,b):
                self.assertAlmostEqual(math.remainder(heading-endpoint[2],2*math.pi),0)
        self.assertGreater(max(abs(np.diff(k))), .1)  # retained G1, explicitly not G2

    def test_closure_spacing_and_conservative_sweeps(self):
        self.assertEqual(self.poses[0], self.poses[-1])
        step=max(math.dist(a[:2],b[:2]) for a,b in zip(self.poses,self.poses[1:]))
        self.assertLessEqual(step,.025)
        self.assertAlmostEqual(self.model.vehicle['disk_radius']+.15+.02,.8166065264130885)
        for margin in (0., .15):
            sweep=_swept_oriented_footprint(self.poses,self.model,margin)
            self.assertTrue(self.model.free.covers(sweep))
            self.assertTrue(Polygon(self.boundary).covers(sweep))
            self.assertEqual(sweep.intersection(self.model.keepout).area,0)
        body=_swept_oriented_footprint(self.poses,self.model,0.)
        self.assertGreater(body.distance(Polygon(self.boundary).boundary),.386)
        self.assertGreater(body.distance(self.model.keepout),1.075)

    def test_fixed_forward_A4_split_and_goal_heading(self):
        splits=a4.closed_loop_splits([p[:2] for p in self.poses])
        self.assertEqual(splits,((0,939),(938,1637)))
        first,second=[self.poses[a:b] for a,b in splits]
        self.assertIs(first[-1],second[0])
        self.assertEqual(first[0],second[-1])
        self.assertAlmostEqual(geometric_curvature(self.poses)[splits[0][1]-1],0)
        for segment in (first,second):
            a,b=segment[-2:]
            self.assertAlmostEqual(math.remainder(math.atan2(b[1]-a[1],b[0]-a[0])-b[2],2*math.pi),0)

    def test_owner_registry_entry_matches_generated_route(self):
        project=dict(map=self.model.identity,chargers=[],regions=[dict(region_id='pillar_v1',display_name='Pillar fixture',route_id='pillar_v1_outer_loop_auto',boundary=self.boundary,options=dict(margin=.15,direction='ccw',laps=1))])
        registry=export_registry(self.model,project)['registry']
        entry=registry['regions'][0]['entry_pose']
        np.testing.assert_allclose([entry[k] for k in ('x','y','yaw')],self.poses[0],atol=1e-12,rtol=0)
        old=json.loads(captured('pillar/original-route.json.gz'))['dense_path_map'][0]
        self.assertAlmostEqual(old['x']-entry['x'],.1904)
        self.assertAlmostEqual(old['y'],entry['y'])
        self.assertEqual(old['yaw'],entry['yaw'])

    def test_legacy_Campus_export_mismatch_is_preserved_and_reproducible(self):
        grid=json.loads(captured('campus/occupancy_grid.json.gz'))
        raw=np.asarray(grid['data']).reshape(grid['height'],grid['width'])
        image=np.array(Image.open(io.BytesIO(captured('campus/map.pgm.gz'))))
        self.assertEqual(int(np.count_nonzero(raw==-1)),52550)
        decoded=decode_trinary(image)
        self.assertEqual(int(np.count_nonzero(decoded==-1)),0)  # old 205/.25 loses all unknown
        self.assertEqual(int(np.count_nonzero(decoded!=raw)),52550)
        expected=np.flipud(np.where(raw<0,127,np.where(raw<=25,255,0)))
        self.assertEqual(int(np.count_nonzero(image!=expected)),77227)
