# Eduexo Experiment (EXO)

This project is designed to control a Dynamixel motor using EEG signals via the Lab Streaming Layer (LSL). The project includes various scripts for setting up the motor, reading EEG signals, and controlling the motor based on these signals.

## Prerequisites

- Python 3.x
- Git
- Visual Studio Code (recommended)
- Compatible hardware for data streaming (e.g., EXO device)
    -Eduexo arm, with Dynamixel motor and its associated hardware (e.g., U2D2, power supply) are required for the experiment.
- Another PC is required to run the main experiment and for LSL communication:
    - Refer to: [https://github.com/JakaHatlakLJ/eduexo_PC]

## Installation

Before running the experiment, ensure you have the following prerequisites:

1. **Clone the repository**:
    ```sh
    git clone [repository-url]
    cd eduexo_EEG
    ```

2. **Lab Streaming Layer (LSL)**:
    Install the LSL library for Python (pylsl==1.17.6). You can install it using pip:
    ```sh
    pip install pylsl
    ```
3. **Setup Dynamixel SDK**:
   Follow the instructions to install and set up the Dynamixel SDK from [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).

4. **Dependencies**:
    All other dependencies are listed in the `requirements.txt` file and need to be installed to your python environment:
    ```sh
    pip install -r requirements.txt
    ```

## Project Hierarchy

The project directory is organized as follows. Once `EXO_main.py` is run for the first time with the default `experiment_configuration.json`, all subfolders will automatically be created and populated if not already


```
eduexo_EEG/
├── frequency_data/                     # Frequency data
├── main/                               # Main scripts and configs
│   ├── EXO_configuration.json          # EXO setup config
│   ├── EXO_LSL.py                      # LSL communication
│   ├── EXO_main.py                     # Run EXO experiment
│   ├── EXO_setup.py                    # Setup EXO system
│   └── Torque_profiles.py              # Generate torque profiles
├── testing_&_debugging/                # Testing and debugging
│   ├── impedance_LSL.py                # Impedance control with LSL
│   ├── LSL_inlet.py                    # LSL inlet handling
│   ├── LSL_send_position.py            # Send position via LSL
│   ├── Pos_Curr_LSL.py                 # Position and current control with LSL
│   ├── Torque_profiles_EXO.py          # Run torque profiles on EXO
│   ├── torque_profiles.png             # Torque profiles image
│   └── Lara/                           # Lara's scripts
│       ├── gravity_comp2.py            # Gravity compensation
│       └── impedance_gravity.py        # Impedance control with gravity
├── requirements.txt                    # Required packages
└── README.md                           # Documentation
```

## Setting Up the Experiment

Before running the `EXO_main.py` script, you need to follow these steps:

1. **Configure and setup PC:**
    - Install all requirements on PC and make sure you can run scripts on it.
    - Refer to: [https://github.com/JakaHatlakLJ/eduexo_PC]

2. **Configure the Experiment:**
    - Open `EXO_configuration.json`.
    - Modify the configuration parameters as needed for your experiment.

## Runing the Experiment

1. Run `experiment_do.py` on PC  to start the experiment. 

2. At the same time run `EXO_main.py` and follow the instructions. (If run for testing purposes you can additionaly use LSL_send_position.py on EXO):
```sh
python EXO_main.py
```

This will start the experiment based on the configurations prepared in the previous steps.

## Additional Information

- Refer to the docstrings and comments within each script for more detailed instructions and explanations.
- The `requirements.txt` file should contain all the necessary Python packages needed to run the project, except pylsl and Dynamixel SDK.

### `EXO_configuration.json` explanation
```json
{
    "torque_limit": 8                           "Maximum torque limit for the motor",
    "max_torque_during_trial": 4                "Maximum torque allowed during a trial",
    "min_pos": 55                               "Minimum position limit for the motor",
    "max_pos": 165                              "Maximum position limit for the motor",
    "DXL_control_mode": 0                       "Control mode for the Dynamixel motor (0 for current control)",
    "baudrate": 1000000                         "Communication baud rate for the motor",
    "control_frequency": 200                    "Frequency at which control commands are sent",
    "duration_of_trials": 3.5                   "Duration of each trial in seconds",
    "incorect_execution_positon_control": 0     "Flag for incorrect execution in position control",
    "incorrect_execution_time_ms": 500          "Time in milliseconds for incorrect execution handling",
    "port_name":                                "/dev/ttyUSB0"  "Port name for the motor connection",
    "frequency_path":                           "./frequency_data"  "Path to save frequency data",
    "save_data": 1                              "Flag to save data (1 to save, 0 not to save)"
}
```

### Additional Scripts:
   1. You can run other scripts for specific functionalities as needed. For example, to run the impedance control with LSL, use:
   ```sh
   python impedance_LSL.py
   ```
   2. For Lara's scripts you migth an additional package (DFRobotLis.py), available [here](https://github.com/DFRobot/DFRobot_LIS).
