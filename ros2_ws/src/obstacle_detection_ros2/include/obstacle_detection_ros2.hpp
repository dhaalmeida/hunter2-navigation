#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/type_adapter.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2_ros/transform_listener.h> //Class
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>//Package: //para fazer o Transform
#include <tf2_ros/buffer.h>
#include <math.h>
#include <cmath>


//usar como "normal"
#define display_measure 4.00//4.50
#define n_setores 21//numeros setores
#define range (210.00*M_PI)/180.00 //range->gama sensorizacao (considerar 210 graus)
#define max_sensor 30.00//30 metros
#define compr 800.00/1000.00 //passar para metros
#define larg 580.00/1000.00   //passar para metros
//para "sensor estacionamento":
#define lateral_points 31
#define longitudinal_points 15
//condicao para enviar dados obstaculos (publicar):
#define pub_obs 1 //se for '1' significa que quer enviar, qql outro valor para nao enviar (ex: 0)



class obstacle_detection_class : public rclcpp::Node {

    protected:
    //SUBSCRIBERS - Receção de dados do CoppeliaSim:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sick_front_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sick_rear_sub;

    //PUBLISHERS
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr sector_obs_pub;
        sensor_msgs::msg::LaserScan sectorsMsgPub;
    
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safe_to_run;
        std_msgs::msg::Bool clear_to_run_pub;  //variavel a publicar

        
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safe_to_nav_SDNL;
        std_msgs::msg::Bool clear_to_navSDNL_pub;  //variavel a publicar
    


            //ros::Publisher parking_sensors_pub; //publicar as distancias [laterais (dir, esq), frente e tras ] em relaçao aos obstaculos: ex: tem obs a 20cm da frente, tem obs a 50cm da lateral direita,....
            //mir_costum_msgs::distance_sector sectorMsgPub; //relativo aos setores
            //sensor_msgs::LaserScan sectorsMsgPub; //ROS 1
            
            //mir_costum_msgs::protect_distance distance_protectionMsgPub;   //relativo as distancias protecao (frente,tras,direita,esquerda)
        int sequ;//para enviar a sequencia de envio 
        //para sensor frontal:
            std::vector<float> front_lidar_data;                         //recebe as 684 distancias do coppelia
            std::vector<float> theta_laser_front;
            std::vector<float> theta_laser_back;
            std::vector<float>distance_front; //distance_front(684,0.0); //as 684 distancias (apos a transformacao)-> mas antes da conversao para os setores dinamicos SDNL
            std::vector<float> angle_front;   //(684,0.0);               //os angulos (apos a transformacao)
            std::vector<float> x_robot_frame_front;        //=0.0;                    //variavel auxiliar (podia-se omitir)
            std::vector<float> y_robot_frame_front;        //=0.0;                    //variavel auxiliar (podia-se omitir)
            //geometry_msgs::PointStamped sick_front_data_in;    //ROS1          //variavel de entrada para a transformacao
            //geometry_msgs::PointStamped sick_front_data_kuka;     //ROS1       //variavel que contem as novas posicoes (apos a transformacao)
            geometry_msgs::msg::PointStamped sick_front_data_in;              //variavel de entrada para a transformacao
            geometry_msgs::msg::PointStamped sick_front_data_kuka;            //variavel que contem as novas posicoes (apos a transformacao)
            //std::vector<float> incre_sensor_front;                       //armazenar os incrementos entre cada laser(calculado a parte no Coppelia)
            float incre_sensor_front;                       //armazenar os incrementos entre cada laser(calculado a parte no Coppelia)
            float min_angle_front;                                       //contem o menor angulo (ocorre reset a cada novo callback ao valor por defeito)
            int front_laser_max_dist;                                    //distancia maxima medivel pelo Lidar
            int front_seq_receive;                                       //para nao armazenar dados atrasados ou repetidos (condicao a verificar na callback)
            tf2_ros::Buffer tfBuffer; //buffer armazena transformada sick frontal
            tf2_ros::TransformListener tfListener;  //{tfBuffer}; //vai colocar no buffer tfBuffer_front tudo o que "ouve"

        //para sensor traseiro:
            std::vector<float> rear_lidar_data; 
            std::vector<float> distance_rear; //(684,0.0); //distancias sensor frente
            std::vector<float> angle_rear;    //(684,0.0); 
            std::vector<float> x_robot_frame_rear;         //=0.0;
            std::vector<float> y_robot_frame_rear;         //=0.0;
            //geometry_msgs::PointStamped sick_rear_data_in;   //ROS1
            //geometry_msgs::PointStamped sick_rear_data_kuka; //ROS1
            geometry_msgs::msg::PointStamped sick_rear_data_in;
            geometry_msgs::msg::PointStamped sick_rear_data_kuka;
            //std::vector<float> incre_sensor_rear;
            float incre_sensor_rear;
            float min_angle_rear;
            int rear_laser_max_dist;
            int rear_seq_receive;
            tf2_ros::Buffer tfBuffer_rear; //buffer armazena transformada sick traseiro
            tf2_ros::TransformListener tfListener_rear;//{tfBuffer_rear}; //vai colocar no buffer tfBuffer_rear tudo o que "ouve" //sera que funciona esta inicializacao???
        
        //distancias aos obstaculos (apenas setores de interesse aos SDNL):
            std::vector<float>dist_obs_setor;

        //variaveis para sistema dinamico:
            std::vector<float> theta_obs;
            std::vector<float> x_setor_final,y_setor_final;
            std::vector<float> robot_dist;//distancia do "corpo do robo" em funcao de cada setor dinamico

        //variavel publicacao:
            bool pub;   //(true==quero publicar)

        //variaveis para fazer o "sensor de estacionamento": (15/03)
            //distancias:
                std::vector<float> dist_front_obs;
                std::vector<float> dist_rear_obs;
                std::vector<float> dist_right_obs;
                std::vector<float> dist_left_obs;
            //angulos:(pelo menos ate 15/03 nao os angulos nao foram utilizados)
                std::vector<float> angle_front_obs;
                std::vector<float> angle_rear_obs;
                std::vector<float> angle_right_obs;
                std::vector<float> angle_left_obs;
            //referencias para calculo:
                std::vector<float> ref_lateral_points;
                std::vector<float> ref_longitudinal_points;
                float distance_between_points_longitudinal;
                float distance_between_points_lateral;
                float min_x_incre;
                float min_y_incre;
        
        //determinar dist. minima:
            float min_lat_left;
            float min_lat_right;
            float min_front;
            float min_rear;
        //Publishing Node
           // rclcpp::Node ObstDetection_Sectors;
    
    public: 
        obstacle_detection_class();  //construtor
        ~obstacle_detection_class(); //destrutor
        bool convertFrameFrontLaser(); //depois talvez colocar como protected???
        bool convertFrameRearLaser();
        bool setup_lasers();
        bool getObs_raw();
        bool getdynamic_obs();
        bool clear_to_start;
        bool setup_obs();
        sensor_msgs::msg::LaserScan pub_obs_data();
        void coppelia_sick_front_Callback(const sensor_msgs::msg::LaserScan::ConstPtr& lidar_front_data);
        void coppelia_sick_rear_Callback(const sensor_msgs::msg::LaserScan::ConstPtr& lidar_rear_data);
        //determinar dist. minima:
            bool min_value_vector_simple(); 
            bool safety_first;
        //Em (25/05):
            bool safety_navSDNL;
            bool publish_sectors;

};

