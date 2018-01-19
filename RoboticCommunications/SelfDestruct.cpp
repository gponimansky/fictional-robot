#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include "MySocket.h"
#include "Pkt_Def.h"

// Declare global bool ExeComplete flag and set to FALSE
bool ExeComplete = false;

void CommandThread(std::string ip, unsigned int port)
{
	// Create a MySocket object configured as a SocketType::CLIENT and ConnectionType::TCP 
	MySocket commandSocket(SocketType::CLIENT, ip, port, ConnectionType::TCP);
	// Establish a reliable connection with the robots command interface 
	commandSocket.ConnectTCP();

	// This process will continue until ExeComplete set to true
	while (!ExeComplete)
	{
		// Query the user in order to get all required information to form a packet, as defined in PktDef (Milestone #1) 
		try
		{
			// Ask what command user wants to form the packet from
			std::cout << ("Enter the command number.\n1. Drive\n2. Arm\n3. Claw\n4. Sleep") << std::endl << std::endl;
			// initialize user variable
			int user = 0;
			// get user data
			std::cin >> user;

			// Define needed variables
			PktDef pkt;
			MotorBody body;

			// Get base the user input
			switch (user)
			{
			// If DRIVE
			case 1:
				pkt.SetCmd(DRIVE);

				// Ask Direction
				std::cout << ("Enter the command number for DRIVE.\n1. Forward\n2. Backward\n3. Right\n4. Left") << std::endl << std::endl;
				std::cin >> user;
				switch (user)
				{
				case 1:
					body.Direction = FORWARD;
					break;
				case 2:
					body.Direction = BACKWARD;
					break;
				case 3:
					body.Direction = RIGHT;
					break;
				case 4:
					body.Direction = LEFT;
					break;
				}
				// Ask duration
				std::cout << ("Enter the duration : ");
				std::cin >> user;

				body.Duration = user;
				pkt.SetBodyData((char *)&body, 2);

				break;
			// If ARM
			case 2:
				pkt.SetCmd(ARM);

				// Ask Direction
				std::cout << ("Enter the command number for ARM.\n1. Up\n2. Down") << std::endl << std::endl;
				std::cin >> user;
				switch (user)
				{
				case 1:
					body.Direction = UP;
					break;
				case 2:
					body.Direction = DOWN;
					break;
				}
				body.Duration = 0;
				pkt.SetBodyData((char *)&body, 2);
				break;
			// If CLAW
			case 3:
				pkt.SetCmd(CLAW);
				// Ask Command
				std::cout << ("Enter the command number for CLAW.\n1. Open\n2. Close") << std::endl << std::endl;
				std::cin >> user;
				switch (user)
				{
				case 1:
					body.Direction = OPEN;
					break;
				case 2:
					body.Direction = CLOSE;
					break;
				}
				body.Duration = 0;
				pkt.SetBodyData((char *)&body, 2);
				break;
			// If SLEEP
			case 4:
				pkt.SetCmd(SLEEP);
				break;
			}
			
			// Increment the PktCount number
			pkt.SetPktCount(pkt.GetPktCount() + 1);
			// Generate the CRC
			pkt.CalcCRC();

			// Transmit the Packet to the robot via the MySocket connection 
			char * buffer = new char[DEFAULT_SIZE];
			commandSocket.SendData(pkt.GenPacket(), pkt.GetLength());

			// Wait for an acknowledgement packet from the robot 
			if (commandSocket.GetData(buffer) < 0)
				throw new std::exception("Data has not been received properly!");
			else if (pkt.CheckCRC(buffer, buffer[5]))
			{
				PktDef newPacket(buffer);
				if (newPacket.GetCmd() == EMPTY)
					throw new std::exception("Negative Acknowledgement Received!");
				else if (pkt.GetCmd() != newPacket.GetCmd())
					throw new std::exception("Command sent is Unequal to Command Received!");
				else if (!newPacket.GetAck())
					throw new std::exception("Ack flag has not been set!");
				else if (newPacket.GetCmd() == SLEEP)
				{
					// This process will continue until the user requests to send a SLEEP command to the robot
					// Disconnect the MySocket 
					commandSocket.DisconnectTCP();
					// Set the bool ExeComplete flag to TRUE 
					ExeComplete = true;
				}
			}

		}
		catch (std::exception e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	// End the thread
}

void TelemetryThread(std::string ip, unsigned int port)
{
	// Create a MySocket object configured as a SocketType::CLIENT and ConnectionType::TCP 
	MySocket telemetrySocket(SocketType::CLIENT, ip, port, ConnectionType::TCP);
	// Establish a reliable connection with the robots command interface 
	telemetrySocket.ConnectTCP();

	// This process will continue until ExeComplete set to true
	while (!ExeComplete)
	{
		try
		{
			// Receive and process all incoming telemetry packets from the Robot.
			char * buffer = new char[DEFAULT_SIZE];
			if (telemetrySocket.GetData(buffer) > 0)
			{
				// Recieve into pkt
				PktDef pkt(buffer);
				// Verification of the CRC && Verification of the Header data 
				if (pkt.CheckCRC(buffer, buffer[5]) && pkt.GetCmd() == STATUS)
				{
					// Display RAW data packet 
					buffer = pkt.GenPacket();
					std::cout << "RAW Data Bytes: ";

					for (int x = 0; x < (int)pkt.GetLength(); x++)
						std::cout << std::hex << (unsigned int)*(buffer++) << ", ";
					// Add a new line and show the dec version of the rest of the data
					std::cout << std::endl << std::dec;

					// Extract and display the sensor data in the body of the packet
					buffer = pkt.GetBodyData();
					short tempData;
					memcpy(&tempData, &buffer[0], 2);
					std::cout << "Sonar Sensor Data: " << tempData << ",  ";
					memcpy(&tempData, &buffer[2], 2);
					std::cout << "Arm Position Data: " << tempData << std::endl;


					std::cout << "Status Bits: " << std::endl;
					char * charData = &buffer[4];
					// Extract and display the Drive flag bit as 0 or 1 
					std::cout << "Drive Flag: " << ((*charData & 0x01) ? "1" : "0") << std::endl;
					// Extract and display the Arm and Claw status in English
					std::cout << "Arm Status: " << ((*charData >> 1 & 0x01) ? "Arm is Up" :
						((*charData >> 2 & 0x01) ? "Arm is Down" : "")) << std::endl;
					std::cout << "Claw Status: " << ((*charData >> 3 & 0x01) ? "Claw is Open" :
						((*charData >> 4 & 0x01) ? "Claw is Closed" : "")) << std::endl << std::endl;
				}
				else
				{
					// If validation fails log the error to the standard out
					std::cout << "Recieved packet is invalid!" << std::endl;
				}
			}
		}
		catch (std::exception e)
		{
			std::cout << e.what() << std::endl;
		}
	}
}

int main(int argc, char* argv[]) {

	// Declare IP Address and Port information variables
	int commandPort, telemetryPort;
	std::string ip;
	
	// Assign the IP Address and Port information variables based on arguments provided
	if (argc == 4)
	{
		ip = argv[1];
		commandPort = (int)argv[2];
		telemetryPort = (int)argv[3];
	}
	else
	{
		// Default
		ip = "127.0.0.1";
		commandPort = 27000;
		telemetryPort = 27501;
	}

	// Spawn and Detach two threads from the main process
	std::thread(CommandThread, ip, commandPort).detach();
	std::thread(TelemetryThread, ip, telemetryPort).detach();

	// Once detached the main process should loop until ExeComplete is TRUE
	while (!ExeComplete) { }
	
	exit(0);
}

