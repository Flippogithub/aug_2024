import yaml
import sys

def validate_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)
            print("YAML is valid!")
            print("Parsed content:")
            print(yaml.dump(config))
    except yaml.YAMLError as e:
        print(f"YAML parsing error: {e}")
        sys.exit(1)

# Replace with your actual file path
validate_yaml('/home/dkflippo/aug2024/src/aug_2024/config/flippo_controllers.yaml')