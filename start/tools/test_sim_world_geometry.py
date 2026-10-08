#!/usr/bin/env python3
import tempfile
import unittest
from pathlib import Path
import numpy as np
from sim_world_geometry import WorldGeometry, box_separation, quaternion_rotation

ROOT=Path(__file__).resolve().parents[2]


class GeometryTests(unittest.TestCase):
    def test_all_world_collision_models_loaded(self):
        for scene,count in [('demo',7),('corridor',8),('3d',8),('blocked',7)]:
            with self.subTest(scene=scene):
                w=WorldGeometry(ROOT/('src/drone_stack/worlds/inspection_'+scene+'.world'))
                self.assertEqual(len(w.boxes),count)
                self.assertTrue(w.check([1,1,1.2])['envelope_clear'])

    def test_demo_column_overlap(self):
        w=WorldGeometry(ROOT/'src/drone_stack/worlds/inspection_demo.world')
        self.assertFalse(w.check([3,1,1.2])['envelope_clear'])
        self.assertTrue(w.check([3,2,1.2])['envelope_clear'])

    def test_corridor_and_boundaries(self):
        w=WorldGeometry(ROOT/'src/drone_stack/worlds/inspection_corridor.world')
        self.assertTrue(w.check([3,1,1.2])['envelope_clear'])
        for p in [[3,.4,1.2],[3,1.6,1.2],[8.7,0,1.2]]:
            self.assertFalse(w.check(p)['envelope_clear'])

    def test_3d_vertical_geometry(self):
        w=WorldGeometry(ROOT/'src/drone_stack/worlds/inspection_3d.world')
        self.assertFalse(w.check([3,1,1.05])['envelope_clear'])
        self.assertTrue(w.check([3,1,1.5])['envelope_clear'])
        self.assertFalse(w.check([4.5,1,2])['envelope_clear'])
        self.assertTrue(w.check([4.5,1,1.5])['envelope_clear'])

    def test_blocked_wall(self):
        w=WorldGeometry(ROOT/'src/drone_stack/worlds/inspection_blocked.world')
        self.assertFalse(w.check([2.8,0,1])['envelope_clear'])

    def test_pose_composition(self):
        sdf='<sdf><world><model name="m"><static>true</static><pose>1 2 0 0 0 1.5707963267948966</pose><link name="l"><pose>2 0 0 0 0 0</pose><collision name="c"><pose>0 1 0 0 0 0</pose><geometry><box><size>1 2 3</size></box></geometry></collision></link></model></world></sdf>'
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'w.sdf';p.write_text(sdf);w=WorldGeometry(p)
        np.testing.assert_allclose(w.boxes[0][1],[0,4,0],atol=1e-12)

    def test_unsupported_geometry_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'w.sdf';p.write_text('<sdf><world><model name="m"><static>true</static><link name="l"><collision name="c"><geometry><sphere><radius>1</radius></sphere></geometry></collision></link></model></world></sdf>')
            with self.assertRaises(ValueError):WorldGeometry(p)

    def test_vectorized_matches_scalar_for_rotations(self):
        rng=np.random.RandomState(27);p=rng.uniform([-8,-8,.5],[8,8,3],size=(500,3))
        q=rng.normal(size=(500,4));q/=np.linalg.norm(q,axis=1)[:,None]
        w=WorldGeometry(ROOT/'src/drone_stack/worlds/inspection_demo.world')
        gaps,ids=w.check_many(p,q)
        for i in range(len(p)):
            scalar=w.check(p[i],q[i])
            self.assertAlmostEqual(gaps[i],scalar['separating_axis_gap_m'],places=10)
            self.assertEqual(w.boxes[ids[i]][0],scalar['closest_obstacle'])


if __name__=='__main__':unittest.main(verbosity=2)
