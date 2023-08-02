from os import path
import logging
import traceback

from .appdirs import user_data_dir
from .settings import MDSTSettings


bl_info = {
    'name': 'MD Spine Tools',
    'author': 'UNOWEN-OwO',
    'version': (0, 0, 1),
    'blender': (3, 6, 0),
    'location': '3D View > UI > MD Spine Tools',
    'description': 'Import Spine mesh and animation form Master Duel',
    'category': 'Import-Export',
    'warning': 'this add-on is beta',
    'project_name': 'md_spine_tools',
}


config_path = path.join(user_data_dir(bl_info['project_name'], False), 'settings.json')
MDST_SETTINGS = MDSTSettings(config_path)

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s:  %(message)s')
MDST_LOGGER = logging.getLogger('md_spine_tools')

try:
    from .mdst_ui import register, unregister
except Exception:
    traceback.print_exc()
