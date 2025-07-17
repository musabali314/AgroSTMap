import app
import os
from datetime import datetime
import yaml

def sort_filenames_by_date(filenames):
    def extract_date(filename):
        # Remove the .pcd extension and parse the date
        return datetime.strptime(filename.replace('.pcd', ''), '%d-%m-%y')

    # Sort using the extracted datetime
    return sorted(filenames, key=extract_date)

# Load configuration from YAML file
with open('config/allign_config.yaml', 'r') as config_file:
    config = yaml.safe_load(config_file)

# Get directory path and colors from config
dir_path = config['directory_path']
colors = config['colors']

filenames = sort_filenames_by_date(os.listdir(dir_path))
file_paths = [os.path.join(dir_path, f) for f in filenames]
app.run(file_paths, colors)
