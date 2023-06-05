#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>

#define DEFAULT_PORT 55555
#define DEFAULT_BUFLEN 512

#define LOOPBACK "127.0.0.0"

const char HOST_IP[] = "192.168.2.10";
bool CONNECTED = false;

int sockfd, n;
char buffer[DEFAULT_BUFLEN];

void attemptSendMap(const nav_msgs::OccupancyGrid::ConstPtr &message)
{
    if (CONNECTED)
    {
        nav_msgs::MapMetaData info = message->info;
        const std::vector<int8_t> incomingMap = message->data;

        if(incomingMap.size() > DEFAULT_BUFLEN) {
            ROS_ERROR("Incoming map is too large for display! ");
            return;
        }

        ROS_INFO("Got Map of %d x %d = %d elements... ", info.width, info.height, incomingMap.size());

        // Fill buffer with data
        for (int i = 0; i < incomingMap.size(); i++)
        {
            if (incomingMap[i] < 0) // If unknown space, keep it 0
                buffer[i] = 0;
            else
                buffer[i] = incomingMap[i];
        }

        ROS_INFO("Attempting to send over socket...");

        // Write data to the socket
        n = write(sockfd, buffer, 300);

        if (n < 0)
        {
            ROS_ERROR("ERROR writing to socket");
        }
        else
        {
            ROS_INFO("Sent %d bytes to host", n);
        }

        bzero(buffer, DEFAULT_BUFLEN); // Clear the buffer

        // Should receive data back from the buffer

        n = read(sockfd, buffer, DEFAULT_BUFLEN);

        if (n < 0)
        {
            ROS_ERROR("ERROR reading from socket");
        }
        else
        {
            ROS_INFO("Got back from server: %s\n", buffer);
        }
    }

    else
    {
        ROS_INFO("ScreenDataTransmitter got map but not connected");
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "screen_controller");
    ros::NodeHandle nodeHandle("~");

    ros::Subscriber mapSubscriber = nodeHandle.subscribe("/tactilemap_with_intent/transformedmap_occupancy", 1, attemptSendMap);

    int portno;

    struct sockaddr_in serv_addr;
    struct hostent *server;

    portno = DEFAULT_PORT;

    // Creating the socket

    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    ROS_INFO("Opening socket... ");

    if (sockfd < 0)
    {
        ROS_ERROR("Unsuccesful!\n");
    }
    else
        ROS_INFO("Success!\n");

    // Try to connect to host

    ROS_INFO("Trying to find host %s on Port %d...", HOST_IP, portno);

    server = gethostbyname(HOST_IP);

    if (server == NULL)
    {
        ROS_ERROR("ERROR, no such host\n");
        exit(0);
    }
    else
        ROS_INFO("Found host\n");

    bzero((char *)&serv_addr, sizeof(serv_addr)); // init serv_addr to zeros
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);

    serv_addr.sin_port = htons(portno);

    while (ros::ok())
    {

        // Connect to the server
        while (!CONNECTED)
        {
            ROS_INFO("Attempting to connect to host... ");

            if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
            {
                ROS_ERROR("ERROR connecting, trying again\n");
                sleep(5);
            }
            else
            {
                ROS_INFO("Connected successfully\n");
                CONNECTED = true;
            }
        }
        // Now connected, keep running until disconnected and then start over trying to connect

        ros::spin();
    }

    // Close the socket
    close(sockfd);
    ROS_INFO("Closed the socket");

    return 0;
};
