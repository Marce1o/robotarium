import yaml 

with open('robot_config.yaml','r') as file:
    robot_config = yaml.safe_load(file)
    rm = robot_config['robomaster']

#print(robot_config)
#print(len(robot_config['robomaster']))
print(rm[f'rm_{1}']['SN'])




