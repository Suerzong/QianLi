from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from frontier_core import Grid
from coverage_core import reachable_domain, map_coverage


class CoverageTests(unittest.TestCase):
    def test_partial_map_uses_fixed_reference_denominator(self):
        ref=Grid(np.zeros((20,40),np.int16),1.,0.,0.)
        spawn=(5.5,5.5,0.)
        domain=reachable_domain(ref,spawn,radius=0.)
        cells=np.full((20,40),-1,np.int16);cells[:,:20]=0
        result=map_coverage(ref,domain,Grid(cells,1.,-5.5,-5.5),spawn)
        self.assertEqual(result['reachable_cells'],800)
        self.assertEqual(result['mapped_free_cells'],400)
        self.assertEqual(result['free_coverage_pct'],50.)

    def test_outside_building_does_not_inflate_coverage(self):
        ref=Grid(np.zeros((20,40),np.int16),1.,0.,0.)
        spawn=(5.5,5.5,0.)
        domain=reachable_domain(ref,spawn,radius=0.,bounds=(0,0,10,20))
        mapped=Grid(np.zeros((100,100),np.int16),1.,-50.,-50.)
        result=map_coverage(ref,domain,mapped,spawn)
        self.assertEqual(result['reachable_cells'],200)
        self.assertEqual(result['mapped_free_cells'],200)

    def test_sealed_room_is_not_a_reachable_target(self):
        cells=np.zeros((20,40),np.int16);cells[:,20]=100
        ref=Grid(cells,1.,0.,0.)
        domain=reachable_domain(ref,(5.5,5.5,0.),radius=0.)
        self.assertTrue(domain[5,5]);self.assertFalse(domain[5,30])
        self.assertEqual(int(domain.sum()),400)

    def test_rotated_spawn_and_map_origin(self):
        ref=Grid(np.zeros((10,10),np.int16),1.,0.,0.)
        spawn=(5.,5.,np.pi/2)
        domain=reachable_domain(ref,spawn,radius=0.)
        mapped=Grid(np.zeros((10,10),np.int16),1.,-5.,5.,-np.pi/2)
        self.assertEqual(map_coverage(ref,domain,mapped,spawn)['free_coverage_pct'],100.)

    def test_false_occupied_cell_does_not_count_as_free_coverage(self):
        ref=Grid(np.zeros((20,40),np.int16),1.,0.,0.)
        spawn=(5.5,5.5,0.);domain=reachable_domain(ref,spawn,radius=0.)
        result=map_coverage(ref,domain,Grid(np.full((20,40),100,np.int16),1.,-5.5,-5.5),spawn)
        self.assertEqual(result['free_coverage_pct'],0.)
        self.assertEqual(result['known_coverage_pct'],100.)


if __name__=='__main__':unittest.main()
