## *********************************************************
##
## File autogenerated for the dynamic_params package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 274, 'description': 'H Max', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'H_Max', 'edit_method': '', 'default': 255, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 274, 'description': 'H Min', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'H_Min', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 274, 'description': 'S Max', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'S_Max', 'edit_method': '', 'default': 255, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 274, 'description': 'S Min', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'S_Min', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 274, 'description': 'V Max', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'V_Max', 'edit_method': '', 'default': 255, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 274, 'description': 'V Min', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'V_Min', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 274, 'description': 'Min Size', 'max': 100000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'Min_Size', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

