# Welcome to M-ARK repo!

This repository contains the **Multi Augmented Reality for Kilobots (M-ARK)** software. If you use this software please cite the following research article:
**XXXXXX** 

# Using a standalone ARK system 
## ARK installation
Download the [ARK](https://github.com/DiODeProject/KilobotArena) source code and follow the provided installation instructions.

## Getting started with ARK experiments 
Follow the [Getting Started with ARK](https://diode.group.shef.ac.uk/kilobots/index.php/Getting_started_on_ARK) instructions.

# Operating multiple physical ARK systems

## Setting up an ARK node as a server

- To run an ARK node as a server you should use the ARK's server experiment code base provided in the ARK_exp_server folder within this repository.
- This experiment code base should be opened and compiled using Qtcreator IDE and then used as any other ARK experiment (see the getting started section above).
- The server node's user must note the IP address of their machine and share it with the client's user. They must also ensure that firewall permissions are in place to allow the client to communicate with the server.
- To start the server, the server node's user must press the "Start Server" button on the ARK's user interface.
- To test that the communication is established between the server and the client, the server's user can write a string in the provided field on the user interface and press "Sent to client". If the communication is working, the client's use should receive the string sent by the server's user.
  
## Setting up an ARK node as a client

We should explain how can we set an ARK node to act as a server.

# Operating physical and simulated ARK systems

## ARGoS installation
- Where to download the ARGoS, Kilobot plugin code from
- How to compile the Kilobot pluging and how to test it

## Setting up an ARK node in ARGoS

## More guidance
