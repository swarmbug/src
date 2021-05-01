# Tools for analysis/data processing

During the project, we created several tools that can assist data analysis and processing tasks. This folder contains such tools, and the below list provides brief descriptions about the tools.

- **Profiling for the configuration definitions** (mentioned in Section 4.1.1)
  - A profiling tool to compute the varnothing (the value that removes an object)
    - [Analysis_varnothing](Analysis_varnothing)
  - A method for identifying configuration variables
    - [Identifying configuration variables](identifying_configuration_variables)
- Understanding impacts of different safety distances (e.g., sensing distances)
  - [A1_safety_dist](A1_safety_dist)
- Generation of test runs for fix validation
  - A1
    - with 20 drones: [A1_validation_large](A1_validation_large)
    - with 4 drones (default): [A1_validation_default](A1_validation_default)
  - A2 and A4: [A2_validation](A2_validation)
  - A3: [A3_validation](A3_validation)
- Test run evaluation
  - [Test_eval](Test_eval)
    - Test run evaluation tool for A2, A3, and A4. Unlike A1, A2~4 is running on the matlab sharing the same process of fuzz.
      It contains comparing using MSE and interpolation (these functionalities for A1 is in its folder separately).
      Note that to use this tool on A2, A3, and A4, customization of input data type is required.
- Generation possible coordinates for simulation before the physical experiment
  - [Simulation_physical](Simulation_physical)
- Crash detection from the traces
  - [crashdetect](crashdetect)
- Data formatting for validation testing
  - [data_processing_validation](data_processing_validation)
- Data preprocessing for SVMap
  - [hull_analysis](hull_analysis)
    - Analysis on outline (convex hull) of distribution of drones' coordinates.
  - [multi_layer](multi_layer)
    - Analysis on the possibility of the drones' locations in a overall distribution.
  - [preprocessing_shift_rotate](preprocessing_shift_rotate)
    - Alignment of individual drones' coordinates based on the flight direction and leader's coordinates.
- Analysis of SVMap
  - [incremental](incremental)
    - Analysis of distribution of drones' coordinates according to configuration of CEP algorithm.
  - [possible_space](possible_space)
    - Analysis of possible spawn locations of drones
- Additional analysis for randomized coordinates as input
  - [randomize_input](randomize_input)
- [Visualization of data](visualization)
