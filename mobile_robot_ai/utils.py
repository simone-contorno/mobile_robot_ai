import json

config_json = open("config.json").read()
config = json.loads(config_json)

# Get the configuration parameter
def get_config_param(parent, child):
    # Parse the JSON configuration
    config_parent = config[parent]
    config_child = config_parent.get(child)
    
    return config_child