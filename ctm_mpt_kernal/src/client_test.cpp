#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include<string.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <tcp_client.h>
#define SERVER_ADDRESS "10.162.94.81"
#define SERVER_PORT    9999
//#define SEND_DATA      "hello world!"
#define recv_Buf_Size  32

int main(int argc, char** argv)
{
        ros::init(argc, argv, "track_path_node");
        ros::NodeHandle nh;
        std::string Address;int Port;
	bool ifgetAddress = ros::param::get("Address", Address);
	bool ifgetPort = ros::param::get("Address", Port);
        if (! ifgetAddress)
                Address = SERVER_ADDRESS;
        if (! ifgetPort)
                Port = SERVER_PORT;
        
        printf("Link Start");
        //创建一个socket
        int clientfd=socket(AF_INET,SOCK_STREAM,0);
        if(clientfd == -1)
        {
                std::cout<<"create client fd error. "<< std::endl;
                return -1;
        }

        //连接服务器
        struct sockaddr_in serveraddr;
        serveraddr.sin_family = AF_INET;
        serveraddr.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
        serveraddr.sin_port = htons(SERVER_PORT);
        if(connect(clientfd,(struct sockaddr *)&serveraddr,sizeof(serveraddr))==-1)
        {
                std::cout << "connect socket error" << std::endl;
                return -1;
        }

        //向服务器端发送数据
        ros::Time begin = ros::Time::now();
        const int N =200;
        int n = 0;
        for (int i = 0;i<N;i++)
        {
        char budd[49];
        Eigen::VectorXf xi(6),di(6);
        xi << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
        di << 0, 0, 0, 0, 0, 0;
        for (int i = 0; i < 6; i++)
            *(float*)(budd + i*4) = xi(i);
        for (int i = 0; i < 6; i++)
            *(float*)(budd + i * 4 + 24) = xi(i);
        
        int ret = send(clientfd, budd, 48, 0);
        //if(ret != strlen(budd))
        //{
        //        std::cout << "send data error." << std::endl;
        //        return -1;
        //}

        std::cout << "send data successfully, data :" << xi.transpose() << std::endl;
        //客户端收数据
        char recvBuf[recv_Buf_Size] = {0};
        ret = recv(clientfd, recvBuf, recv_Buf_Size, 0);
        if(ret >0)
        {
                for (int i = 0; i < 6;i++)
                {
                    di(i) = *(float*)(recvBuf + i * 4);
                }
                std::cout << "recv data successfully, data: " << di.transpose() <<std::endl;
            
        }
        else
        {
                for (int i = 0; i < 6;i++)
                {
                    di(i) = *(float*)(recvBuf + i * 4);
                }
                std::cout << "recv data error, data: " << di.transpose() <<std::endl;
        }
        ros::Time end = ros::Time::now();
        if ((end - begin).toSec() > 1)
                break;
        n++;
        }
        
        std::cout << "Hz: " << n << std::endl;
        //关闭socket
        close(clientfd);

        return 0;
}