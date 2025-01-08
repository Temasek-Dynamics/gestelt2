# Python Venv guide
Context: some packages do not run on Python 3.12 and require older versions of python e.g. pytransform3d, therefore we have to use venvs to run them.

```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update && sudo apt dist-upgrade
sudo apt install python3.11
sudo apt install python3.11-venv
mkdir pytransform_venv && cd pytransform_venv
python3.11 -m venv env
source env/bin/activate
```

## Requirements
1. numpy: 1.26.4
2. open3d: 0.18.0 


Reference:
1. https://gist.github.com/AlexMakesSoftware/6d5592b1c2974041eecc71e6ed1cdd1b