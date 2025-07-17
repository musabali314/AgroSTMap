# Spatio-Temporal Mapping

This project provides tools for spatio-temporal mapping, including NDT and GICP registration modules, and visualization utilities.

## Installation

### Dependencies

To set up the Python bindings, ensure the following dependencies are installed:

- **PCL**: Version 1.8 or higher
- **Eigen3**: Required for linear algebra operations
- **pybind11**: Install via `pip3 install pybind11`
- **Python**: Version 3.8 or higher (other versions may work but are untested)

Install the dependencies using the following commands:

```bash
sudo apt-get update
sudo apt-get install libpcl-dev libeigen3-dev python3-pip
pip3 install pybind11
```

## Registration Modules

### NDT Registration Module

To install the NDT registration module:

```bash
cd 3rd-party/ndt_omp
sudo python3 setup.py install
```

### GICP Registration Module

To install the GICP registration module:

```bash
cd 3rd-party/fast_gicp
python3 setup.py install --user
```

## Result Visualization

For visualization, install the required dependencies listed in [requirements.txt](requirements.txt). It is recommended to use an Anaconda environment for managing dependencies.

### Setup Anaconda Environment

1. Download and install [Anaconda](https://www.anaconda.com/download).
2. Create and activate a new environment with Python 3.10:

```bash
conda create -n stm_env python=3.10
conda activate stm_env
pip install -r requirements.txt
```

### Running Visualizations

#### Alignment Visualization

1. Configure the alignment settings in [config/align_config.yaml](config/align_config.yaml).
2. Run the alignment visualization:

```bash
python align_vis.py
```

#### Growth Visualization

1. Configure the growth visualization settings in [config/growth_vis.yaml](config/growth_vis.yaml).
2. Run the growth visualization:

```bash
python growth_vis.py
```
