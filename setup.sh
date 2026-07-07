printf "\n----- SETTING UP PYTHON EVNIORNMENT -----\n\n"
VENV_DIR="quadSimPythonVenv/QSVenv"
# Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment in $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi
source "$VENV_DIR/bin/activate"
echo "Installing dependencies from requirements.txt..."

# Install pip if not present
if ! command -v pip >/dev/null 2>&1; then
    echo "pip not found, installing..."
    python3 -m ensurepip --upgrade
else
    echo "pip already present, skipping install ($(pip --version))"
fi
printf "\nEnvironment successfully created at $VENV_DIR\n"


printf "\n----- SETTING UP BUILD ENVIORNMENT -----\n\n"
mkdir build
cd build
cmake ..
make -j


printf "\n----- ALL DONE :) -----\n\n"

