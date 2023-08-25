# Welcome to M-ARK repo!

This repository contains the **Multi Augmented Reality for Kilobots (M-ARK)** software. If you use this software please cite the following research article:
**XXXXXX** 

# Using a standalone ARK system 
## ARK installation
Download the [ARK](https://github.com/DiODeProject/KilobotArena) source code and follow the provided installation instructions.

## Getting started with ARK experiments 
Follow the [Getting Started with ARK](https://diode.group.shef.ac.uk/kilobots/index.php/Getting_started_on_ARK) instructions.

# Operating multiple physical ARK systems

The following describes the procedure to execute the DHTF experiment, presented in the paper, within M-ARK.

## Setting up an ARK node as a server

- To run an ARK node as a server you should use the ARK's server experiment code base provided in the ARK_exp_server folder within this repository.
- This experiment code base should be opened and compiled using Qtcreator IDE and then used as any other ARK experiment (see the getting started section above).
- The server node's user must note the IP address of their machine and share it with the client's user. They must also ensure that firewall permissions are in place to allow the client to communicate with the server.
- To start the server, the server node's user must press the "Start Server" button on the ARK's user interface.
- To test that the communication is established between the server and the client, the server's user can write a string in the provided field on the user interface and press "Sent to client". If the communication is working, the client's use should receive the string sent by the server's user.
  
## Setting up an ARK node as a client

- To run an ARK node as a server you should use the ARK's client experiment code base provided in the ARK_exp_client folder within this repository.
- This experiment code base should be opened and compiled using Qtcreator IDE and then used as any other ARK experiment (see the getting started section above).
- The client node's user must know the IP address of the server machine.
- To connect to the server, the client node's user must select or manually insert the IP address of the server machine and press the "Connect to Server" button on the ARK's user interface.

## Kilobot behaviour
The code to be uploaded to the Kilobot is the same for both the experiments. Upload the `kilobot_ALF_dhtf.c` behaviour located at `M-ARK_ARGOS/src/examples/behaviors/kilobot_ALF_dhtf.c`. Be careful that line 9 of that file is **commented**.  

# Operating simulated ARK systems

## ARGoS installation
- Make sure you have [ARGoS 3.0.0-beta56](https://github.com/ilpincy/argos3/releases/tag/3.0.0-beta56) installed!
- Make sure you have the [kilobot plug-in](https://github.com/ilpincy/argos3-kilobot) installed!


## Setting up an ARK node in ARGoS
- Copy the M-ARK_ARGoS folder content in the folder where the kilobot plug-in is installed. 
- **NOTE:** keep the same folder structure and update all the CMakeLists.txt in a coherent manner.
- **NOTE:** Be careful that line 9 of `M-ARK_ARGOS/src/examples/behaviors/kilobot_ALF_dhtf.c` file is **uncommented**.

### Compiling the code

```shell
cd <PATH_TO_KILOBOT_PLUG-IN>
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../src
make
sudo make install
```

### Execute DHTF
Open two terminals, and launch the following commands in the same order.

On the first terminal launch:
```shell
argos3 -c src/examples/experiments/kilobot_ALF_dhtf_server.argos
```

On the second terminal launch:
```shell
argos3 -c src/examples/experiments/kilobot_ALF_dhtf_client.argos
```

### Execute CRE
Open two terminals, and launch the following commands in the same order.

On the first terminal launch:
```shell
argos3 -c src/examples/experiments/kilobot_ALF_cre_server.argos
```

On the second terminal launch:
```shell
argos3 -c src/examples/experiments/kilobot_ALF_cre_client.argos
```

# Operating physical and simulated ARK systems
M-ARK can be also executed in mixed modality. Follow the procedure described before to run a server node of M-ARK, and replace the IP address at which the client node, executed with ARGoS, tries to connect. The parameter to change is the `ip_addr` within the `kilobot_ALF_dhtf_client.argos` files.

# Final notes
- CRE experiment here is provided only in simulation.
- CRE experiment has been tested in mixed mode, but it is not reported here.
