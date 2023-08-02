import os
import json
import logging
import os.path as path


SETTINGS_LOG = logging.getLogger("md_spine_tools.settings")


class MDSTSettings(object):
    def __init__(self, filepath):
        if path.exists(filepath):
            # read settings file
            self.load_settings_file(filepath)
        else:
            # new settings file
            try:
                os.makedirs(path.dirname(filepath))
                with open(filepath, 'w') as _:
                    pass
            except OSError:
                SETTINGS_LOG.error('Failed creating new settings file', exc_info=True)

        # default settings
        self.config_path = filepath
        self.config_dir = path.dirname(filepath)
    
    def __setattr__(self, name, value):
        super(MDSTSettings, self).__setattr__(name, value)
        self.save_settings_file()

    def __getattr__(self, attr):
        try:
            return super(MDSTSettings, self).__getattr__(attr)
        except AttributeError:
            return None

    def __delattr__(self, name):
        super(MDSTSettings, self).__delattr__(name)
        self.save_settings_file()

    def load_settings_file(self, filepath):
        settings_dict = {}
        with open(filepath) as f:
            try:
                settings_dict = json.load(f)
            except Exception:
                SETTINGS_LOG.error('Failed loading settings file', exc_info=True)

        self.config_path = filepath
        for k, v in settings_dict.items():
            setattr(self, k, v)

    def save_settings_file(self):
        # save settings file
        with open(self.config_path, 'w') as f:
            try:
                json.dump(self.__dict__, f, indent=4)
            except Exception:
                SETTINGS_LOG.error('Failed saving settings file', exc_info=True)
