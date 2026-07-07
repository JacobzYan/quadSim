printf "\n----- SETTING UP PYTHON EVNIORNMENT -----\n\n"
VENV_DIR="quadSimPythonVenv/QSVenv"
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment in $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi
source "$VENV_DIR/bin/activate"
echo "Installing dependencies from requirements.txt..."
pip install --upgrade pip
pip install -r quadSimPythonVenv/requirements.txt
echo "Environment successfully created at $VENV_DIR"


printf "\n----- SETTING UP BUILD ENVIORNMENT -----\n\n"
mkdir build
cd build
cmake ..
make -j


printf "\n----- ALL DONE :) -----\n\n"

