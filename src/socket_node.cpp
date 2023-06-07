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

const std::string map_topic = "/tactilemap_with_intent/outputmap_occupancy";

void attemptSendMap(const nav_msgs::OccupancyGrid::ConstPtr &message)
{
    // First check if we are connected, if not, cannot do anything
    if (CONNECTED)
    {
        nav_msgs::MapMetaData info = message->info;
        const std::vector<int8_t> incomingMap = message->data;

        ROS_INFO("Socket node got Map of %d x %d = %d elements... ", info.width, info.height, incomingMap.size());

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
        // n = read(sockfd, buffer, DEFAULT_BUFLEN);
        // if (n < 0)
        // {
        //     ROS_ERROR("ERROR reading from socket");
        // }
        // else
        // {
        //     ROS_INFO("Got back from server: %s\n", buffer);
        // }
    }

    else
    {
        ROS_INFO("But socket is not connected");
    }
}

// void mapToBytes(const std::vector<int8_t> inVector, uint8_t * buffer, int buflen){

//     uint8_t dataContainer[300] = {0};
//     int curcell = 0;

//     // Loop through cells, first by row, then by column
//     for (int bc_y=0; bc_y<10; bc_y++){

//         for(int bc_x=0; bc_x<30; bc_x++){
//             // This code runs for each individual cell
// //            std::printf("%3d ", curcell);

//             // Start coordinates for this cell
//             int x_start = bc_x*2;
//             int y_start = bc_y*4;

// //            std::printf("{%2d,%2d} \t", x_start, y_start);

//             // Loop through each of the dots of this cell
//             for(int x=x_start; x<x_start+2; x++){   // First loop through left column downwards, then right

//                 for(int y=y_start; y<y_start+4; y++){

//                     uint8_t curPixel = inVector[y][x];
//                     if (curPixel > 1) {
//                         throw std::invalid_argument("Input vector must only contain 1's or 0's");
//                     }

//                     uint8_t curBitLoc = (x-x_start)*4 + (y-y_start);  // If in second row, add 4 to y to obtain location
//                     outData[curcell] |= curPixel << (7-curBitLoc);
// //                    std::printf("(%d)%d ", curBitLoc,curPixel);
//                 }

//             }

// //            std::printf("\t evaluates to %d\n", outData[curcell]);
//             curcell ++;
//         }
//     }
    
// }

int main(int argc, char **argv)
{

    ros::init(argc, argv, "screen_controller");
    ros::NodeHandle nodeHandle("~");

    ros::Subscriber mapSubscriber = nodeHandle.subscribe(map_topic, 1, attemptSendMap);

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
