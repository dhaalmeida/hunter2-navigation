#include <obstacle_detection_ros2.hpp>
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

//obstacle_detection_class::obstacle_detection_class(){ //construtor
obstacle_detection_class::obstacle_detection_class()
: tfBuffer(this->get_clock()), tfBuffer_rear(this->get_clock()), tfListener_rear(tfBuffer_rear), tfListener(tfBuffer), Node("ObstacleDetection")
{
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile.reliability(rclcpp::ReliabilityPolicy::SystemDefault);

    //SUBSCRIBERS
    sick_front_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scanner/scan", qos_profile, std::bind(&obstacle_detection_class::coppelia_sick_front_Callback, this, _1)); 
    sick_rear_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("lidar_rear_data", 1, std::bind(&obstacle_detection_class::coppelia_sick_rear_Callback, this, _1));

    //PUBLISHER
    auto sector_obs_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("sector_dist_obs",1);
    sectorsMsgPub = sensor_msgs::msg::LaserScan();

      
    auto safe_to_run = this->create_publisher<std_msgs::msg::Bool>("deep_safe_distance",10);
    clear_to_run_pub.data=true;  //variavel a publicar

    auto safe_to_nav_SDNL = this->create_publisher<std_msgs::msg::Bool>("keep_safe_distance_SDNL",10);
    clear_to_navSDNL_pub.data = true;  //variavel a publicar
    
    publish_sectors=true;

        
        
    //inicializar variaveis auxiliares para tratamento dados dos Lidar recebidos do Coppelia:
        //para sensor frente:
            front_lidar_data.assign(684,0.0);
            //distance_front.assign(684,0.0);
            //angle_front.assign(684,0.0);
            //x_robot_frame_front.assign(684,0.0);
            //y_robot_frame_front.assign(684,0.0);
            //incre_sensor_front.assign(684,0.0);
            front_seq_receive=0;
            sick_front_data_in.header.frame_id="velodyne";
        //para sensor tras:
            rear_lidar_data.assign(684,0.0);
            //distance_rear.assign(684,0.0);
            //angle_rear.assign(684,0.0);
            //x_robot_frame_rear.assign(684,0.0);
            //y_robot_frame_rear.assign(684,0.0);
            //incre_sensor_rear.assign(684,0.0);
            rear_seq_receive=0;
            sick_rear_data_in.header.frame_id="sick_rear";
            //tfListener_rear(tfBuffer_rear){}; //Nao se faz isto em class's...

        //distancias aos obstaculos (apenas setores de interesse aos SDNL):
            dist_obs_setor.assign(n_setores,30.0); //todos os setores a "detetar" 30m (no inicio)

    //inicializar variaveis auxiliares:
        clear_to_start=false;   //so iniciar o processo caso a Callback seja executada 1x
        //para converter em setores:
            theta_obs.assign(n_setores,0.0);
            x_setor_final.assign(n_setores,0.0);
            y_setor_final.assign(n_setores,0.0);
        //ter distancia do corpo do robo a descontar na distancia medida em cada setor dinamico
            robot_dist.assign(n_setores,0.0);

        //variavel publicacao:
            pub=true;// publicar os obstaculos para o modulo de  (true==quero publicar)

    //variaveis para fazer o "sensor de estacionamento": (15/03)
            //distancias:
                dist_front_obs.assign(longitudinal_points,30.00); //by default is 30 m
                dist_rear_obs.assign(longitudinal_points,30.00);
                dist_right_obs.assign(lateral_points,30.00);
                dist_left_obs.assign(lateral_points,30.00);
            //angulos:
                angle_front_obs.assign(longitudinal_points,0.00); //by default is 0 rad
                angle_rear_obs.assign(longitudinal_points,0.00);
                angle_right_obs.assign(lateral_points,0.00);
                angle_left_obs.assign(lateral_points,0.00);
            //referencias para calculo:
                ref_lateral_points.assign(lateral_points,0.00); 
                ref_longitudinal_points.assign(longitudinal_points,0.00);
                distance_between_points_longitudinal= larg/(longitudinal_points-1); //frente ou tras
                distance_between_points_lateral= compr/(lateral_points-1);          //laterais (dir ou esq)
                min_x_incre=distance_between_points_lateral/2.0;
                min_y_incre=distance_between_points_longitudinal/2.0;

                for(int aux=0;aux<longitudinal_points;aux++){ //para frente e tras do veiculo
                    ref_longitudinal_points[aux]=(aux*distance_between_points_longitudinal)-(larg/2);
                    //ROS_INFO("pontos frente/tras referencia: para %d e de %f",aux,ref_longitudinal_points[aux]);
                }

                for(int aux=0;aux<lateral_points;aux++){ //para laterais do veiculo (esq e dir)
                    ref_lateral_points[aux]=(aux*distance_between_points_lateral)-(compr/2);
                    //ROS_INFO("pontos laterais referencia: para %d e de %f",aux,ref_lateral_points[aux]);
                }

            //para enviar dados ao miar_functions: (16/03)
                //setor_obs_pub = nh.advertise<mir_costum_msgs::distance_sector>("sector_dist_obs",1); //publicar obs para SDNL

                //sector_obs_pub = nh.advertise<sensor_msgs::LaserScan>("sector_dist_obs",1);//ROS1 //publicar obs para SDNL
                
                
                //parking_sensors_pub = nh.advertise<mir_costum_msgs::protect_distance>("protec_dist_obs",1); //publicar para definir distancia seguranca a volta do veiculo
                sequ=0;//sequencia começa em 0


           //determinar dist. minima:
            min_lat_left=0.0;
            min_lat_right=0.0;
            min_front=0.0;
            min_rear=0.0;
            safety_first=true;

          
}

obstacle_detection_class::~obstacle_detection_class(){
    //destructor

}
/* sensor_msgs::LaserScan:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
*/

//Callback que recebe os dados em bruto do sensor frontal(do Coppelia)
void obstacle_detection_class::coppelia_sick_front_Callback(const sensor_msgs::msg::LaserScan::ConstPtr& msg_coppelia_sick_front){
   
  //  if(msg_coppelia_sick_front->header.seq >= front_seq_receive){//se a msg recebida nao e antiga ou repetida:
        RCLCPP_INFO(this->get_logger(),"-----------------------------SENSOR FRENTE-----------------------------");
        front_lidar_data=msg_coppelia_sick_front->ranges;
        //incre_sensor_front=msg_coppelia_sick_front->intensities; //o vector 'intensities' foi utilizado para enviar os incrementos (a tropa manda desenrascar xD)
        incre_sensor_front = msg_coppelia_sick_front->angle_increment; 
        min_angle_front=msg_coppelia_sick_front->angle_min;      //!!!Pode dar asneira: se o tempo de execucao disto for superior a taxa de rececao!!!
        front_laser_max_dist=msg_coppelia_sick_front->range_max;

       // front_seq_receive=msg_coppelia_sick_front->header.seq; //atualizar sequencia msg recebida
  //  }
    clear_to_start=true;
}

//Callback que recebe os dados em bruto do sensor traseiro(do Coppelia)
void obstacle_detection_class::coppelia_sick_rear_Callback(const sensor_msgs::msg::LaserScan::ConstPtr& msg_coppelia_sick_rear){
   
   //if(msg_coppelia_sick_rear->header.seq >= rear_seq_receive){
        RCLCPP_INFO(this->get_logger(),"-----------------------------SENSOR TRASEIRO-----------------------------");
        rear_lidar_data=msg_coppelia_sick_rear->ranges;
        //incre_sensor_rear=msg_coppelia_sick_rear->intensities; //o vector 'intensities' foi utilizado para enviar os incrementos (a tropa manda desenrascar xD)
        incre_sensor_rear = msg_coppelia_sick_rear->angle_increment;
        min_angle_rear=msg_coppelia_sick_rear->angle_min;      //!!!Pode dar asneira: se o tempo de execucao disto for superior a taxa de rececao!!!
        rear_laser_max_dist=msg_coppelia_sick_rear->range_max;
        
     //   rear_seq_receive=msg_coppelia_sick_rear->header.seq;
   //}
}

bool obstacle_detection_class::setup_lasers(){

    float lidar_angle_front=min_angle_front;
    theta_laser_front.clear();
    theta_laser_back.clear();
    //ROS_INFO("Tamanho array %d",front_lidar_data.size());

    if(front_lidar_data.size()==0)
        return false;

    for(int i=0;i<front_lidar_data.size();i++)
    {
        theta_laser_front.push_back(lidar_angle_front);
        //ROS_INFO("sector %d angle %f",i,theta_laser_front[i]);
        lidar_angle_front = lidar_angle_front+incre_sensor_front;
    }
    float lidar_angle_back=min_angle_rear;
    for(int i=0;i<rear_lidar_data.size();i++)
    {
        theta_laser_back.push_back(lidar_angle_back);
        lidar_angle_back = lidar_angle_back + incre_sensor_rear;
    }

    return true;
}


//funcao que converte dados entre o referencial do sensor(frente) e o da plataforma
bool obstacle_detection_class::convertFrameFrontLaser(){
    //float angle=min_angle_front;
    //sick_front_data_in.header.stamp=ros::Time(); //ROS1
    sick_front_data_in.header.stamp=rclcpp::Time();

    distance_front.clear();
    angle_front.clear();
    x_robot_frame_front.clear();
    y_robot_frame_front.clear();

    //ROS_INFO("Angle_inicial = %f",angle);

    //angle = angle + incre_sensor_front*31;
    //ROS_INFO("Depois de alterado Angle_inicial = %f",angle);


    //for(int pos=1;pos<684;pos++){
    for(int pos=31;pos<633;pos++){
        //ROS_INFO("Frente: Setor %d: dist= %f angle = %f",pos,rear_lidar_data[pos],theta_laser_front[pos]*180/M_PI);
        //converter em posicoes cartesianas:
        sick_front_data_in.point.x=cos(theta_laser_front[pos])*front_lidar_data[pos];
        sick_front_data_in.point.y=sin(theta_laser_front[pos])*front_lidar_data[pos];

        //listener_obsfront.waitForTransform("MirRef_handler", "sick_front", ros::Time(0), ros::Duration(1.0));

        try{
            //tfBuffer.transform(sick_front_data_in, sick_front_data_kuka, "mirref_handler",ros::Duration(1.0)); //fica preso ate 1s ate que chegue novo broadcast (maximo)
            tfBuffer.transform(sick_front_data_in, sick_front_data_kuka, "velodyne",tf2::Duration(std::chrono::seconds(1)));
            //armazenar as novas posicoes(apos a transformacao):
            /*x_robot_frame_front[pos]=(sick_front_data_kuka.point.x);
            y_robot_frame_front[pos]=(sick_front_data_kuka.point.y);

            distance_front[pos]=(sqrt(pow(sick_front_data_kuka.point.x, 2) + pow(sick_front_data_kuka.point.y, 2))); //distancia

            //angle_front[b]=atan2(y_robot_frame_front, x_robot_frame_front);//angulo
            angle_front[pos]=(atan2(sick_front_data_kuka.point.y, sick_front_data_kuka.point.x));*/


            x_robot_frame_front.push_back(sick_front_data_kuka.point.x);
            y_robot_frame_front.push_back(sick_front_data_kuka.point.y);
            distance_front.push_back(sqrt(pow(sick_front_data_kuka.point.x, 2) + pow(sick_front_data_kuka.point.y, 2)));
            angle_front.push_back(atan2(sick_front_data_kuka.point.y, sick_front_data_kuka.point.x));

            //ROS_INFO("Frente conversão: Distance %f: angle= %f ",distance_front[distance_front.size()-1],angle_front[distance_front.size()-1]*180/M_PI);


            //ROS_INFO("Laser %d tem angulo %f (em graus %f)",pos,angle_front[pos],(angle_front[pos]*180.00)/3.1415);//Para debug!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            //codigo utilizado como debug das distancias lidas do sick, em relacao ao sensor e ao frame do robo
/*                if(distance_front[pos]<display_measure){ //so para apresentar os setores de que detetam alguma coisa util
                ROS_INFO("-------------------Novo setor-------------------");
             //   ROS_INFO("Sick setor %d, esta em x= %f e y= %f", b, sick_front_data_in.point.x,sick_front_data_in.point.y);
             //   ROS_INFO("Frame Robo setor %d, esta em x= %f e y= %f", b, sick_front_data_kuka.point.x,sick_front_data_kuka.point.y);
                ROS_INFO("FRENTE: Setor %d, tem distancia de %f e angulo de %f", pos,distance_front[pos],(angle_front[pos]*180.00)/3.1415);
                }*/
        }
        catch(tf2::TransformException& ex){
            RCLCPP_INFO(this->get_logger(),"Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }
        //angle=angle+incre_sensor_front;
    }
    return true;
}

//funcao que converte dados entre o referencial do sensor(traseiro) e o da plataforma
bool obstacle_detection_class::convertFrameRearLaser(){
    //float angle=min_angle_rear;
    //sick_rear_data_in.header.stamp=ros::Time(); //ROS1
    sick_rear_data_in.header.stamp=rclcpp::Time();
    distance_rear.clear();
    angle_rear.clear();
    x_robot_frame_rear.clear();
    y_robot_frame_rear.clear();

    //angle = angle + incre_sensor_rear*29;

    //for(int pos=1;pos<684;pos++){
    for(int pos=29;pos<646;pos++){
        //converter em posicoes cartesianas:
        //ROS_INFO("Tras: Setor %d, dist= %f angle = %f ",pos,rear_lidar_data[pos],theta_laser_back[pos]*180/M_PI);
        sick_rear_data_in.point.x=cos(theta_laser_back[pos])*rear_lidar_data[pos];
        sick_rear_data_in.point.y=sin(theta_laser_back[pos])*rear_lidar_data[pos];

        try{
            //tfBuffer.transform(sick_rear_data_in, sick_rear_data_kuka, "mirref_handler",ros::Duration(1.0)); //ROS1 //fica preso ate 1s ate que chegue novo broadcast (maximo)
            tfBuffer.transform(sick_rear_data_in, sick_rear_data_kuka, "map",tf2::Duration(std::chrono::seconds(1)));
            //armazenar as novas posicoes(apos a transformacao):
            /*x_robot_frame_rear[pos]=(sick_rear_data_kuka.point.x);
            y_robot_frame_rear[pos]=(sick_rear_data_kuka.point.y);

            distance_rear[pos]=(sqrt(pow(sick_rear_data_kuka.point.x, 2) + pow(sick_rear_data_kuka.point.y, 2))); //distancia
            
            angle_rear[pos]=(atan2(sick_rear_data_kuka.point.y, sick_rear_data_kuka.point.x));*/

            x_robot_frame_rear.push_back(sick_rear_data_kuka.point.x);
            y_robot_frame_rear.push_back(sick_rear_data_kuka.point.y);
            distance_rear.push_back(sqrt(pow(sick_rear_data_kuka.point.x, 2) + pow(sick_rear_data_kuka.point.y, 2)));
            angle_rear.push_back(atan2(sick_rear_data_kuka.point.y, sick_rear_data_kuka.point.x));

            //ROS_INFO("Trás conversão: Distance %f: angle= %f ",distance_rear[distance_rear.size()-1],angle_rear[distance_rear.size()-1]*180/M_PI);
            
             //codigo utilizado como debug das distancias lidas do sick, em relacao ao sensor e ao frame do robo
/*                if(distance_rear[pos]<display_measure){ //so para apresentar os setores de que detetam alguma coisa util
                    ROS_INFO("-------------------Novo setor-------------------");
             //   ROS_INFO("Sick setor %d, esta em x= %f e y= %f", b, sick_front_data_in.point.x,sick_front_data_in.point.y);
             //   ROS_INFO("Frame Robo setor %d, esta em x= %f e y= %f", b, sick_front_data_kuka.point.x,sick_front_data_kuka.point.y);
                    ROS_INFO("Traseiro: Setor %d, tem distancia de %f e angulo de %f", pos,distance_rear[pos],(angle_rear[pos]*180.00)/3.1415);
                }*/
        }
        catch(tf2::TransformException& ex){
            RCLCPP_INFO(this->get_logger(),"Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }
        //angle=angle+incre_sensor_rear;
    }
    return true;
}

bool obstacle_detection_class::getObs_raw(){
    //ROS_INFO("Dados sensor frontal:");
    convertFrameFrontLaser();
  /*  for(int i=0;i<angle_front.size();i++) {
        ROS_INFO("Imprimir debug(fr) x= %f e y= %f", x_robot_frame_front[i],y_robot_frame_front[i]);
        ROS_INFO("Para i=%d, angulo e de %f rad (%f graus)",i,angle_front[i],(angle_front[i]*180.00)/M_PI);
        ROS_INFO("Angulo  deveria de ser %f",(atan2(y_robot_frame_front[i],x_robot_frame_front[i])));
    }*/
    //ROS_INFO("Dados sensor traseiro:");

    //////////////////////////////////////////////////////////////////////////////////////////////convertFrameRearLaser();

    /*for(int i=0;i<angle_rear.size();i++){
        ROS_INFO("Imprimir debug(re) x= %f e y= %f", x_robot_frame_rear[i],y_robot_frame_rear[i]);
        ROS_INFO("Para i=%d, angulo e de %f rad (%f graus)",i,angle_rear[i],(angle_rear[i]*180.00)/M_PI);
        ROS_INFO("Angulo  deveria de ser %f",(atan2(y_robot_frame_rear[i],x_robot_frame_rear[i])));
    } */      
return true;
}

//esta funcao determina os angulos de cada setor dinamico (para os SDNL) em funcao do nº setores desejados e a gama de sensorizacao
//A funcao tambem calcula a distancia a "descontar" em por cada setor (considerar o proprio corpo do robo)
//NOTA: !!  O nº de setores TEM de ser um NUMERO IMPARE  !!
bool obstacle_detection_class::setup_obs(){

    float aux_inc=range/(n_setores-1.00);
    float alfa=atan2(larg,compr);  //tem de ser atan2   !!!
    //ROS_INFO("Angulo alfa e de %f", alfa); //para debug

    for(int i=0;i<n_setores;i++){
        theta_obs[i]=-(range/2.0)+aux_inc*i;            //definir os angulos que correspondem aos setores de interesse aos sistemas dinâmicos (n_setores/2 tem de corresponder a 0 graus (frente robo))
        x_setor_final[i]=max_sensor*cos(theta_obs[i]);  //calcular ponto 'x' maximo do setor 'i'
        y_setor_final[i]=max_sensor*sin(theta_obs[i]);  //calcular ponto 'y' maximo do setor 'i'
        //ROS_INFO("Setor dinamica %d para angulo de %f (em graus: %f)",i, theta_obs[i], (theta_obs[i]*180.00)/M_PI);
        
        //caso os setores pertençam à parte frontal ou traseira do veiculo:
        //este tem de ter modulo (pois passa por -pi!!):                        AQUI                AQUI
        if((-alfa<=theta_obs[i] && theta_obs[i]<=alfa) || ((M_PI-alfa)<=fabs(theta_obs[i]) && fabs(theta_obs[i])>=(alfa-M_PI))){//    (-29,14<theta<29,14) ou (+150,86<theta<-150,86)
            robot_dist[i]=fabs((compr/2.00)/cos(theta_obs[i]));
            //ROS_INFO("Tras ou frente do robo. A distancia corpo robo e de %f metros",robot_dist[i]);
//em caso de duvidas, ver caderno na data de 10/03/2022!!!!!
        }
        //caso os setores pertençam às laterais do veiculo:
        else if((alfa<=theta_obs[i] && theta_obs[i]<=(M_PI-alfa)) || ((alfa-M_PI)<=theta_obs[i] && theta_obs[i]<=(-alfa))){
            robot_dist[i]=fabs((larg/2.00)/sin(theta_obs[i]));
            //ROS_INFO("Laterais do robo. A distancia corpo robo e de %f metros",robot_dist[i]);
        }

    }

return true;
}

bool obstacle_detection_class::getdynamic_obs(){

    float angle_min=0.0;
    float angle_max=0.0;
    float x_intersecao=0.0;
    float y_intersecao=0.0;
    float m_reta=0.0;   //declive da reta entre dois lasers consecutivos do sick
    float b_reta=0.0;   //so a reta entre dois lasers consecutivos do sick tem 'b' (os setores saem do (0,0) do eixo central do veiculo)
    float m_setor=0.0;  //declive setor dinamico
    float distance=0.0;
    dist_obs_setor.assign(n_setores,30.00);

    getObs_raw();//para fazer as transformacoes entre referenciais.

    //sensor frente (considerar todos os lasers-> nenhum deteta o proprio veiculo):
    //comecar por '1' e nao por '0' pois na 1ª iteracao precisa do valor do angulo anterior!!!
    for(int f=1;f<=angle_front.size();f++){  //'f' stands for 'front'
//iniciado a 11/02
        
        if(angle_front[f-1]<=angle_front[f]){ //tem que ter estas condicoes pois os angulos nao variam linearmente com os setores (pois os sick estao na lateral e a 45º do centro do veiculo)
            angle_max=angle_front[f];
            angle_min=angle_front[f-1];
        }
        else{   //caso contrario    (ex o setor 200 tem 30º e o setor 201 tem 29º devido à disposicao do obstaculo)
            angle_max=angle_front[f-1];
            angle_min=angle_front[f];
        }

//nao confundir o 'pos' e o 'f'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        for(int pos=0;pos<n_setores;pos++){ //percorrer todos os setores dinâmicos definidos

            if(angle_min<theta_obs[pos] && theta_obs[pos]<angle_max){ //se o angulo do setor esta dentro entre os dois "lasers" do sick:
              
                if((x_robot_frame_front[f]-x_robot_frame_front[f-1])==0.00){  //condicao 1 (ver caderno dia 08/03)
                    x_intersecao=x_robot_frame_front[f];
                    m_setor=y_setor_final[pos]/x_setor_final[pos]; //ou m_setor=sin(theta_obs[pos])/cos(theta_obs[pos]); ou ainda m_setor=tan(theta_obs[pos]);
                    y_intersecao=x_intersecao*m_setor;
                }
                else if(x_setor_final[pos]==0.00){ //condicao 2 -> quando theta_obs[pos]==90º ou -90º (ver caderno dia 08/03)
                    x_intersecao=0.0; //esta sobreposta ao eixo dos xx
                    m_reta=(y_robot_frame_front[f]-y_robot_frame_front[f-1])/(x_robot_frame_front[f]-x_robot_frame_front[f-1]);
                    b_reta=y_robot_frame_front[f]-m_reta*x_robot_frame_front[f];
                    y_intersecao=b_reta;
                }
                else{ //condicao 3 -> o "normal" (ver caderno dia 08/03)
                    m_setor=(y_setor_final[pos]/x_setor_final[pos]);
                    m_reta=((y_robot_frame_front[f]-y_robot_frame_front[f-1])/(x_robot_frame_front[f]-x_robot_frame_front[f-1]));
                    b_reta=y_robot_frame_front[f]-m_reta*x_robot_frame_front[f];
                    x_intersecao=(b_reta/(m_setor-m_reta));
                    y_intersecao=m_setor*x_intersecao;
                }

                distance=(sqrt(pow(x_intersecao, 2)+pow(y_intersecao, 2)))-robot_dist[pos]; //ja retirar a distancia do "corpo" do veiculo
                //if(distance<1)
                //{
                //    ROS_INFO("angle_max = %f angle_min = %f distance = %f dist1 = %f dist2 = %f",angle_max,angle_min,distance,distance_front[f],distance_front[f-1]);
                //}
                //porquê utilizar o 'distance>0.0'? -> Não esquecer que estamos a subtrair o "corpo" do robo, mas em principio nao deveria de ser, se acontecer algo de errado aconteceu...
                if(dist_obs_setor[pos]>distance && distance>0.00){//se a ultima distancia armazenada no setor 'pos' for maior do que a nova, ATUALIZAR COM A MENOR DISTANCIA!!!!!!!
                    dist_obs_setor[pos]=distance;
                    //ROS_INFO("O setor e o %d e o laser e %d", pos,f);
                    //ROS_INFO("Frente: Entrei para Theta= %f. Angle_min=%f Angle_max=%f. Dist=%f",theta_obs[pos],angle_min,angle_max,distance);
                    /*if(distance<0.50){//so para debug....
                        ROS_INFO("O setor e o %d e o laser e %d", pos,f);
                        ROS_INFO("Posicionamento de x= %f e y=%f.  A distancia a descontar do robo e de %f",x_intersecao,y_intersecao,robot_dist[pos]);
                        }*/
                }
                //acho que nao e preciso meter o b_reta,m_reta,m_setor a 0, pois quando sao utilizados antes sao calculados
            }
        }
    }

    for(int r=1;r<=angle_rear.size();r++){  //'r' stands for 'rear'
//iniciado a 11/02
        if(angle_rear[r-1]<angle_rear[r]){ //tem que ter estas condicoes pois os angulos nao variam linearmente com os setores (pois os sick estao na lateral e a 45º do centro do veiculo)
            angle_max=angle_rear[r];
            angle_min=angle_rear[r-1];
        }
        else{   //caso contrario    (ex o setor 200 tem 30º e o setor 201 tem 29º devido à disposicao do obstaculo)
            angle_max=angle_rear[r-1];
            angle_min=angle_rear[r];
        }
        //(ver caderno dia 15/03 para ver a razao deste if....)
        if(angle_max-angle_min<M_PI){ //precisa deste 'if'caso dois lasers do lidar detetassem obstaculo em, p.e. -175º e outro em +175º a gama seria de 350º xD (esta solucao apenas e util caso nao utilizar sensores traseiros para a dinamica SDNL)


//nao confundir o 'pos' e o 'f'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            for(int pos=0;pos<n_setores;pos++){ //percorrer todos os setores dinâmicos "impostos"

                if(angle_min<theta_obs[pos] && theta_obs[pos]<angle_max){ //se o angulo do setor esta dentro entre os dois "lasers" do sick:

                    if((x_robot_frame_rear[r]-x_robot_frame_rear[r-1])==0.00){  //condicao 1 (ver caderno dia 08/03)
                        x_intersecao=x_robot_frame_rear[r];
                        m_setor=y_setor_final[pos]/x_setor_final[pos]; //ou m_setor=sin(theta_obs[pos])/cos(theta_obs[pos]); ou ainda m_setor=tan(theta_obs[pos]);
                        y_intersecao=x_intersecao*m_setor;
                    }
                    else if(x_setor_final[pos]==0.00){ //condicao 2 -> quando theta_obs[pos]==90º ou -90º (ver caderno dia 08/03)
                        x_intersecao=0.0; //esta sobreposta ao eixo dos xx
                        m_reta=(y_robot_frame_rear[r]-y_robot_frame_rear[r-1])/(x_robot_frame_rear[r]-x_robot_frame_rear[r-1]);
                        b_reta=y_robot_frame_rear[r]-m_reta*x_robot_frame_rear[r];
                        y_intersecao=b_reta;
                    }
                    else{ //condicao 3 -> o "normal" (ver caderno dia 08/03)
                        m_setor=(y_setor_final[pos]/x_setor_final[pos]);
                        m_reta=((y_robot_frame_rear[r]-y_robot_frame_rear[r-1])/(x_robot_frame_rear[r]-x_robot_frame_rear[r-1]));
                        b_reta=y_robot_frame_rear[r]-m_reta*x_robot_frame_rear[r];
                        x_intersecao=(b_reta/(m_setor-m_reta));
                        y_intersecao=m_setor*x_intersecao;
                    }

                    distance=(sqrt(pow(x_intersecao, 2)+pow(y_intersecao, 2)))-robot_dist[pos]; //ja retirar a distancia do "corpo" do veiculo
                    //porquê utilizar o 'distance>0.0'? -> Não esquecer que estamos a subtrair o "corpo" do robo, mas em principio nao deveria de ser, se acontecer algo de errado aconteceu...
                    if(dist_obs_setor[pos]>distance && distance>0.00){//se a ultima distancia armazenada no setor 'pos' for maior do que a nova, ATUALIZAR COM A MENOR DISTANCIA!!!!!!!
                        dist_obs_setor[pos]=distance;
                        //ROS_INFO("Dinamica obs TRAS: setor %d, laser %d. Theta= %f, min_theta=%f max_theta=%f",pos,r,theta_obs[pos],angle_min,angle_max);
                    }
                }
            }
        }
    }    
    RCLCPP_INFO(this->get_logger(),"Finalizei a conversao....");
    //verificacao:
    for(int i=0;i<n_setores;i++){
        RCLCPP_INFO(this->get_logger(),"Setor %d: dist= %f theta= %f (em graus %f)",i,dist_obs_setor[i],theta_obs[i],(theta_obs[i]*180.00)/M_PI);
    }

    return true;
}

//funcao para enviar os dados dos obstaculos para a dinamica de navegacao:
sensor_msgs::msg::LaserScan obstacle_detection_class::pub_obs_data(){



   // sectorsMsgPub.header.seq = sequ;
    sectorsMsgPub.ranges = dist_obs_setor;
    sectorsMsgPub.angle_min = theta_obs.front();
    sectorsMsgPub.angle_max = theta_obs.back();
    sectorsMsgPub.angle_increment = theta_obs[1]-theta_obs[0];
    sectorsMsgPub.range_min = 0.0;
    sectorsMsgPub.range_max = 30.0;
    sectorsMsgPub.intensities = theta_obs;
    sectorsMsgPub.ranges = dist_obs_setor;
    //RCLCPP_INFO(this->get_logger(),"Método de Publicação");
    
   
    //setor_obs_pub.publish(sectorMsgPub);  
    //sector_obs_pub.publish(sectorsMsgPub); //ROS1                       //publicar a mensagem relativa aos setores
   auto ObstDetection_Sectors = rclcpp::Node::make_shared("ObstDetectionPub"); //2025
   sector_obs_pub = ObstDetection_Sectors->create_publisher<sensor_msgs::msg::LaserScan>("sector_dist_obs",1); //2025

    sector_obs_pub->publish(sectorsMsgPub); 
    
    sequ++; //por fim incrementar a seq. de envio

return sectorsMsgPub;
}


//verificar distancias minimas em 360º do veiculo. Publica 'false' caso o veiculo tenha que parar por seguranca
bool obstacle_detection_class::min_value_vector_simple(){
    min_lat_left = *min_element(dist_left_obs.begin(), dist_left_obs.end());
    min_lat_right = *min_element(dist_right_obs.begin(), dist_right_obs.end());
    min_rear = *min_element(dist_rear_obs.begin(), dist_rear_obs.end());
    min_front = *min_element(dist_front_obs.begin(), dist_front_obs.end());

    auto ObstDetection_Sectors = rclcpp::Node::make_shared("ObstDetection_INFO"); //TESTEEEE
    auto safe_to_run = ObstDetection_Sectors->create_publisher<std_msgs::msg::Bool>("deep_safe_distance",10); /////////////////////////////////  TESTE
    auto safe_to_nav_SDNL = ObstDetection_Sectors->create_publisher<std_msgs::msg::Bool>("keep_safe_distance_SDNL",10); ///////////////////////////

    if(min_lat_left<0.30 || min_lat_right<0.30 || min_rear<0.30 || min_front<0.30){ //parte navegacao 
        clear_to_navSDNL_pub.data = false;
        //safe_to_nav_SDNL.publish(clear_to_navSDNL_pub); //ROS1


       // this->safe_to_nav_SDNL->publish(clear_to_navSDNL_pub);
       safe_to_nav_SDNL->publish(clear_to_navSDNL_pub);
        
        if(min_lat_left<0.10 || min_lat_right<0.10 || min_rear<0.10 || min_front<0.20){ //parte da navegacao omnidirecional
            clear_to_run_pub.data=false;
         
           // this->safe_to_run->publish(clear_to_run_pub);  //parar!!
            safe_to_run->publish(clear_to_run_pub);  //parar!!
            RCLCPP_INFO(this->get_logger(),"Mandar parar a navegacao omnidirecional");
            return false;
        }
    }
    else{
        clear_to_navSDNL_pub.data = true;

        //this->safe_to_nav_SDNL->publish(clear_to_navSDNL_pub);
        safe_to_nav_SDNL->publish(clear_to_navSDNL_pub);
        clear_to_run_pub.data=true;

        
        //this->safe_to_run->publish(clear_to_run_pub); 
        safe_to_run->publish(clear_to_run_pub); 
        return true; 
    }
    return true;
}


int main(int argc, char **argv)
{  
    //ros::init(argc, argv, "obstacle_detection"); //ROS1
    //ros::NodeHandle n_priv("~");                 //ROS1
    rclcpp::init(argc,argv);    //ROS2
    auto node = std::make_shared<obstacle_detection_class>(); //ROS2
    rclcpp::Rate rate(20);

    //ros::Rate rate(20); //ROS1  //20Hz (50ms)  //80Hz (12,5ms)
    

    while(!(node->clear_to_start)){ //fica a espera que o 'positioning' fique operacional
        
        //ROS_INFO("Ainda a espera do Positioning"); //ROS1
        //ros::spinOnce(); //ROS1
        RCLCPP_INFO(node->get_logger(),"Ainda a espera do Positioning");
        rclcpp::spin_some(node);
		rate.sleep();
    }
    //Create Publisher Node
    //sensor_msgs::msg::LaserScan sectors;
    
    //definir os angulos dos setores dinamicos:
    node->setup_obs();
    node->setup_lasers();

    
    while(rclcpp::ok())
    { 
        node->getdynamic_obs();

        if(node->publish_sectors==1){//se for para enviar os dados
           // RCLCPP_INFO(node->get_logger(),"antes de publicar");

            node->pub_obs_data();  //publica os dados dos sensores (nos topicos 'sector_dist_obs' e 'protec_dist_obs')
            
           // RCLCPP_INFO(node->get_logger(),"depois de publicar");
        }
       node->safety_first=node->min_value_vector_simple();

        //ROS_INFO("Verificacao obstaculos");//ROS1
        RCLCPP_INFO(node->get_logger(),"Verificacao obstaculos");

        if(node->safety_first){
            //ROS_INFO("Seguro Para andar"); //ROS1
            RCLCPP_INFO(node->get_logger(),"Seguro Para andar");
        }
        else{
            //ROS_INFO("Obstaculos demasiado proximos");  //ROS1
            RCLCPP_INFO(node->get_logger(),"Verificacao obstaculos");
        }

        //ros::spinOnce(); //ROS 1
        rclcpp::spin_some(node);
	    rate.sleep();
    }
    rclcpp::shutdown();
}