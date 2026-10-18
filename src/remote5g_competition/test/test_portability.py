#!/usr/bin/env python3
"""Exercise installed launch setup under another HOME and outside the workspace."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

PACKAGE = Path(get_package_share_directory('remote5g_competition'))
SIM = Path(get_package_share_directory('remote5g_sim'))
sys.path.insert(0, str(PACKAGE / 'scripts'))
# Load the documented optional cache before changing HOME for the path test.
sys.path.append(str(Path.home() / '.cache/remote5g_scene_libs'))
from scene_core import load_config


def launcher(package, filename):
    spec = importlib.util.spec_from_file_location('portable_launch', package / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations['gui'] = 'false'
    return module, context


class PortabilityTest(unittest.TestCase):
    def test_installed_launch_with_another_home_and_spaces(self):
        with tempfile.TemporaryDirectory(prefix='remote5g user ') as temp:
            # No writes to either the installation or the original user's reports.
            with patch.dict(os.environ, {'HOME': temp}):
                module, context = launcher(PACKAGE, 'competition.launch.py')
                output = Path(context.launch_configurations['output'])
                self.assertEqual(output, Path(temp) / '.ros/remote5g/reports/competition/current')
                module.setup(context)
                manifest = json.loads((output / 'layout.json').read_text())
                self.assertEqual(Path(manifest['config']['workspace']), (SIM / 'track').resolve())
                self.assertTrue((output / 'competition.world').is_file())
                # Gazebo does not decode %20 in file URIs; use model resources instead.
                for uri in ET.parse(output / 'competition.world').findall('.//uri'):
                    parsed = urlsplit(uri.text)
                    self.assertEqual(parsed.scheme, 'model', uri.text)
                    models = (SIM / 'track/simulation/models' if parsed.netloc.startswith('wa_400m_track_')
                              else output / 'models')
                    resource = models / parsed.netloc / unquote(parsed.path.lstrip('/'))
                    self.assertTrue(resource.exists(), uri.text)
                # Track scenes must use installed resources, including copied installs.
                for scene in ('flat', 'red', 'blue'):
                    module, context = launcher(SIM, 'sim.launch.py')
                    context.launch_configurations['scene'] = scene
                    self.assertEqual(context.launch_configurations['workspace'], str(SIM / 'track'))
                    self.assertTrue(module.setup(context))

    def test_explicit_workspace_and_relative_config(self):
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp) / 'config.yaml'
            config.write_text('workspace: custom_track\n')
            self.assertEqual(load_config(config)['workspace'], str(Path(temp) / 'custom_track'))
            self.assertEqual(load_config(config, {'workspace': str(SIM / 'track')})['workspace'],
                             str((SIM / 'track').resolve()))

    def test_shipped_model_resources(self):
        models = list((PACKAGE / 'models').rglob('*.sdf'))
        models += [p for p in (PACKAGE / 'worlds').rglob('*') if p.suffix in ('.sdf', '.world')]
        for model in models:
            resources = next(parent / 'models' for parent in model.parents
                             if (parent / 'models').is_dir())
            for uri in ET.parse(model).findall('.//model//uri'):
                parsed = urlsplit(uri.text)
                self.assertEqual(parsed.scheme, 'model', uri.text)
                resource = resources / parsed.netloc / unquote(parsed.path.lstrip('/'))
                self.assertTrue(resource.exists(), uri.text)


if __name__ == '__main__':
    unittest.main()
