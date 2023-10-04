#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include<string.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#define SERVER_ADDRESS "10.162.94.81"
#define SERVER_PORT    9999
const size_t N_DIM = 6;
class Tcp_Client_Socket
{
private:

public:

    Tcp_Client_Socket(std::string Address, int Port);
    Eigen::Matrix<float, N_DIM, 1> Contorl_Once(Eigen::Matrix<float, 6, 1> Err,Eigen::Matrix<float, 6, 1> xi);
    Send_Train_Message()
};