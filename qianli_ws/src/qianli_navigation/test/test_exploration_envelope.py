from pathlib import Path
import ast, importlib.util, math, unittest
import yaml

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('navigation_launch', root/'launch/navigation.launch.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


class EnvelopeTests(unittest.TestCase):
    def test_every_edge_contains_required_clearance_disk(self):
        params = yaml.safe_load((root/'config/nav2_params.yaml').read_text())
        result = module.exploration_parameters(params, .47)
        for name in ('local_costmap','global_costmap'):
            cfg = result[name][name]['ros__parameters']
            disk = .47+(.05/math.sqrt(2)+.005 if name=='global_costmap' else 0.)
            points = ast.literal_eval(cfg['footprint'])
            for a,b in zip(points, points[1:]+points[:1]):
                distance = abs(a[0]*b[1]-a[1]*b[0])/math.hypot(b[0]-a[0],b[1]-a[1])
                self.assertGreaterEqual(distance, disk)
            # Narrowest training doors are 1.2 m; this envelope still fits.
            self.assertLess(max(math.hypot(*p) for p in points), .6)
            self.assertEqual(cfg['footprint_padding'], 0.)

    def test_live_scan_cannot_override_unknown_static_space(self):
        params = yaml.safe_load((root/'config/nav2_params.yaml').read_text())
        result = module.exploration_parameters(params, .47)
        for name in ('local_costmap','global_costmap'):
            cfg = result[name][name]['ros__parameters']
            self.assertIn('static_layer', cfg['plugins'])
            self.assertTrue(cfg['track_unknown_space'])
            self.assertEqual(cfg['obstacle_layer']['combination_method'], 2)


if __name__ == '__main__':unittest.main()
