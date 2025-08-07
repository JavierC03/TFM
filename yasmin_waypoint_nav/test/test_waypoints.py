import unittest
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class TestYasminWaypointNav(unittest.TestCase):

    def test_waypoints_file_exists(self):
        """Test que el archivo de waypoints existe"""
        waypoints_path = os.path.join(
            get_package_share_directory('yasmin_waypoint_nav'),
            'config',
            'waypoints.yaml'
        )
        self.assertTrue(os.path.exists(waypoints_path))

    def test_waypoints_file_format(self):
        """Test que el archivo de waypoints tiene el formato correcto"""
        waypoints_path = os.path.join(
            get_package_share_directory('yasmin_waypoint_nav'),
            'config',
            'waypoints.yaml'
        )
        
        with open(waypoints_path, 'r') as f:
            waypoints = yaml.safe_load(f)
        
        self.assertIsInstance(waypoints, list)
        self.assertGreater(len(waypoints), 0)
        
        # Verificar que cada waypoint tiene las claves requeridas
        for wp in waypoints:
            self.assertIn('x', wp)
            self.assertIn('y', wp)
            self.assertIn('z', wp)
            self.assertIn('w', wp)
            
            # Verificar que los valores son numéricos
            self.assertIsInstance(wp['x'], (int, float))
            self.assertIsInstance(wp['y'], (int, float))
            self.assertIsInstance(wp['z'], (int, float))
            self.assertIsInstance(wp['w'], (int, float))


if __name__ == '__main__':
    unittest.main()
