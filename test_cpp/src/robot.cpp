#ifndef ROBOT__CPP
#define ROBOT__CPP

#include "robot.h"

using namespace std;


    
        Robot::Robot(){
            //vuoto
        };

        Robot::Robot(bool init,int argc, char ** argv){//da chiamare una volta per non avere errori
            if(init){
                joint.position={0.0,0.0,0.0,0.0,0.0,0.0};
            }
            int i=1;
            ros::init(argc,argv, "robot_cpp");
            ros::NodeHandle n;
            //ros::Publisher js_pub = n.advertise < sensor_msgs::JointState > ("/command", 1);
            //Robot::js_pub = n.advertise < sensor_msgs::JointState > ("/command", 1);
            //Robot::js_pub = n.advertise < sensor_msgs::JointState > ("/ur5/joint_group_pos_controller/command", 1);
            js_pub = n.advertise < std_msgs::Float64MultiArray > ("/ur5/joint_group_pos_controller/command", 1);
            grip_pub = n.advertise < std_msgs::Float64 > ("/gripper", 1);
        };
/*! @brief base constructor
*needs argc and argv to innitialize the joint publisher
*/
        Robot::Robot(int argc, char ** argv){
            cout << "in robot" << endl;

            std::stringstream ss;
            ss << "hello world ";
            msg.data = ss.str();
            //std_msgs::String msg = you_initial_value_here;
            /*
            ros::init(argc, argv, "robot_cpp");
            ros::NodeHandle n;
            ros::Rate loop_rate(1);

            //ros::AsyncSpinner spinner(0);
            //spinner.start();


            //sub = n.subscribe("/ur5/joint_states", 1, js_callback);
            ros::Subscriber sub = n.subscribe("chatter", 10, robot_msg_callback);
            */
            cout << "done constructor" << endl;
        };

        void Robot::spin(){
            ros::spinOnce();
        }
/*! @brief subscribes and starts callbacks
* subscribes to /ur5/joint_states and starts the callbash
*/
        void Robot::start_callback(int argc, char ** argv){
            cout << "in robot" << endl;
            ros::init(argc, argv, "robot_cpp_callback");
            ros::NodeHandle n;
            ros::Rate loop_rate(1);

            //ros::AsyncSpinner spinner(0);
            //spinner.start();
            //sub = n.subscribe("/ur5/joint_states", 1, js_callback);
            //ros::Subscriber sub = n.subscribe("chatter", 10, robot_msg_callback);
            ros::Subscriber sub_js = n.subscribe("/ur5/joint_states", 1, js_callback);

            cout << "start spinning robot" << endl;
            ros::MultiThreadedSpinner spinner(2);
            spinner.spin();
            cout << "done spinning robot" << endl;
        }
/*! @brief publishes a joint configuration
*
* @param th joitn configuration to publish
* @param fill for previous version debugging
* @param gripper for previous version debugging
*/

        int Robot::publish(Eigen::Vector < double, 6 > th, bool fill, double gripper){
            
            std_msgs::Float64MultiArray f64j;
            sensor_msgs::JointState joint_tmp;

            //std::cout << "\npublishing...\n" << std::endl;

            //f64j.data.empty();

            for(int i=0; i<6;i++){
                //joint.position[i] = th[i];
                //std::cout << "\npublishing... " << th[i] << std::endl;
                f64j.data.push_back(th[i]);
                //std::cout << "\nstacked..." << std::endl;
            }

            //f64j.data.push_back(gripper);
            //f64j.data.push_back(gripper);
            
            //per soft gripper

            for(int i=0; i< Robot::n_grip;i++){ //pos 1 2 (3)
                //f64j.data.insert ( std::next(f64j.data.begin()) , gripper );//posizione 2
                f64j.data.push_back( Robot::gripper_j[i] ); // attacco lo stato corrente del gripper
            }

            //std::cout << "argc: " << argc << "\nargv: " << argv << std::endl;
            //std::cout << "joint to publish: " << joint << std::endl;
            js_pub.publish(f64j);

            //std::cout << "\published!!\n" << std::endl;

            return 0;
        }
/*! @brief publishes grip joints in simulation \n
* and publishes on /gripper the opening value
* @param gripper gripper joint value/opening size
*/
        int Robot::publish_grip(double gripper){
            
            std_msgs::Float64MultiArray f64j;
            sensor_msgs::JointState joint_tmp;
            std_msgs::Float64 f64;

            f64.data=gripper;
            grip_pub.publish(f64);

            //f64j.data.empty();

            for(int i=0; i<6;i++){
                f64j.data.push_back(Robot::joint.position[i]);//pubblico la stessa posizione
            }

            //print_f64j(f64j);

            //cout << "\nn_grip: " << Robot::n_grip << endl;

            for(int i=0; i < Robot::n_grip ; i++){ //pos [6] [7] [8]

                if(Robot::gripper_j.empty())
                    cout << "\ngripper vuoto\n";
                //cout << "\nset gripper to: "<< gripper;
                //f64j.data.insert ( std::next(f64j.data.begin()) , gripper );//posizione [1]
                //f64j.data.push_back( double(rand()%int(M_PI*100))/100 -  (M_PI/2) );
                f64j.data.push_back( gripper );
            }

            cout << "\n";

            //print_f64j(f64j);

            js_pub.publish(f64j);

            return 0;
        }

/*! @brief rotates the end effector
*
* @param ang in radiants
*/
        int Robot::rotate(double ang){
            sensor_msgs::JointState j_now = joint;
            Eigen::Vector<double, 6> j_tmp = j_to_q(j_now);
            j_tmp(5)=ang;
            publish(j_tmp);
            return 0;
        }
/*! @brief prints a float64 array
*
* @param f64j ros float 64 array
*/
        void Robot::print_f64j(std_msgs::Float64MultiArray f64j){
            for(int i=0; i< f64j.data.size();i++){
                cout << "\nf64j.data[" << i << "] " << f64j.data[i];
            }
            cout << "\n\n";
        }
        /*
        int publish(sensor_msgs::JointState& joint_2, bool fill = true, double gripper = 0){

            std_msgs::Float64MultiArray f64j;
            Eigen::Vector < double, 6 > th;

            //if(fill){
            if(false){
                //std::cout << "filling with 0" << std::endl;
                joint_2.velocity={0.0,0.0,0.0,0.0,0.0,0.0};
                joint_2.effort={0.0,0.0,0.0,0.0,0.0,0.0};
            }

            for(int i=0; i<6;i++){
                th(i) = joint.position[i];
            }

            //std::cout << "argc: " << argc << "\nargv: " << argv << std::endl;
            //std::cout << "joint to publish: " << joint << std::endl;
            //js_pub.publish(joint_2);

            publish(th,fill);

            return 0;
        }
        */
/*! @brief moves only the shoulder joint for the robot \n 
* used to reduce the chance of collision of the robot arm
* @see float move_to(Eigen::Vector < double, 6 > pr_f, int steps = 3000, float k_coeff=0.01, int verbose=false)
*/
        float Robot::move_to_shoulder(Eigen::Vector < double, 6 > pr_f, int steps, float k_coeff, bool verbose){
            Kin kin;
            Eigen::Vector < double, 6 > q_i = j_to_q(joint);//q di adesso
            Eigen::Vector < double, 6 > tmp = kin.compute_ik(pr_f)[Kin::ik_index];//q di dove voglio andare
            Eigen::Vector < double, 6 > q_f;
            q_f=q_i;//stessa posizione
            q_f(0)=tmp(0);//ma con shoulder pan finale
            kin.compute_fc(q_f);
            Eigen::Vector < double, 6 > pr_pan=kin.get_pr_now();
            move_to(pr_pan,steps,k_coeff,verbose);

            return 0;
        }
/*! @brief moves the robot to a given position \n
* calculates the path from the current position and \n 
* the final one, publishes the joints for the manipulator
* 
* @see Kin::p2p()
* @see js_callback(const sensor_msgs::JointState& j)
* @param pr_f position and rotation of the final position
* @param steps number of 1ms steps
* @param k_coeff for debugging and compatibility
* @param verbose verbose option
*/
        float Robot::move_to(Eigen::Vector < double, 6 > pr_f, int steps, float k_coeff, bool verbose){
            Helper help;
            Eigen::MatrixXd k = Eigen::MatrixXd::Identity(6,6) * k_coeff;
            sensor_msgs::JointState j_now = joint;
            Kin kin;
            float f;
            Eigen::Vector < double, 6 > pr_i;//stato attuale
            //Eigen::Vector < double, 6 > pr_f;

            //help.fill_pr_i_f(pr_i,pr_f);//prendo il punto finale
            Eigen::Vector < double, 6 > q = j_to_q(j_now);//punto iniziale
            //q = help.constrainAngle180(q);//180-180
            
            kin.compute_fc(q);
            pr_i=kin.get_pr_now();
            /*
            pr_i(0) = kin.get_ee_p()[0];
            pr_i(1) = kin.get_ee_p()[1];
            pr_i(2) = kin.get_ee_p()[2];
            pr_i(3) = kin.rotm2eul(kin.T0e)[0];
            pr_i(4) = kin.rotm2eul(kin.T0e)[1];
            pr_i(5) = kin.rotm2eul(kin.T0e)[2];//sovrascrivo con la posizione attuale
            */
            

            cout << "moving from: " << pr_i << endl << endl;
            cout << "moving to: " << pr_f << endl << endl;

            cout << "moving from q: " << q << endl << endl;
            cout << "moving to q: " << pr_f << endl << endl;

            //vector<Eigen::Vector < double, 6 >> path_theory = kin.fillpath(q,pr_f, steps);
            //vector<Eigen::Vector < double, 6 >> path = kin.da_a(path_theory,q,k,steps);
            vector<Eigen::Vector < double, 6 >> path = kin.p2p(pr_i,pr_f,q,steps);
            /*
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
            path[0]=j_to_q(joint);
            cout << "joints attuali:" << q << endl<<endl;
            cout << "path[0]" << path[0] << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
            */
            ofstream myfile;
            /*
            
            myfile.open ("path_p_theo.txt");
            for (Eigen::Vector < double, 6 > i: path_theory)
                    myfile << i(0)<< "," <<i(1)<< "," <<i(2)<< "," <<i(3)<< "," <<i(4)<< "," <<i(5)<< "\n";
            myfile.close();
            */
            
            myfile.open ("path.txt");
            for (Eigen::Vector < double, 6 > i: path)
                    myfile << i(0)<< "," <<i(1)<< "," <<i(2)<< "," <<i(3)<< "," <<i(4)<< "," <<i(5)<< "\n";
            myfile.close();
            
            //cout << "\npath dim: "<< path.size()<<"\n";

            ros::Rate loop_rate(1000);
            int path_i=0;

            while(ros::ok && path_i<steps){
        
                //sleep(0.001);
                //cout << path_i << ")\n" << std::flush;;

                //Robot::print_position(Robot::joint);
                //cin >> f;
                publish(path[path_i]);
                //path_i=steps;
                //ros::sleep(0.2);
                //cout << path_i << "\t";
                path_i++;
                //cout << "loop" << path_i << " ";
                loop_rate.sleep();
                //sleep(0.001);
            }


            //print_position();
            //cout << "\nsteps fatti: " << path_i << endl;
            if(verbose){

                float delta;
                Eigen::Vector3d p_theo;
                Eigen::Vector3d p_real;
                Eigen::Vector < double, 6 > q_verbose;
                //kin.compute_fc(pr_f);//finale teorica
            
                p_theo << pr_f[0],pr_f[1],pr_f[2];//finale teorico
                q_verbose = j_to_q(Robot::joint);//prendo la posizione reale
                kin.compute_fc(q_verbose);//pos reale
                p_real= kin.get_ee_p();

                delta = help.dist(p_theo,p_real);

                cout << "teorico: " << p_theo << endl;
                cout << "reale: " << p_real << endl;
                cout << "delta: " << delta << endl;

            }
            


            sleep(0.3);//serve


            return 0;
        }

        Eigen::Vector < double, 6 > Robot::j_to_q(sensor_msgs::JointState j_tmp){
            Eigen::Vector < double, 6 > q_tmp;
            for(int i=0; i< 6; i++)
                q_tmp[i]=j_tmp.position[i];
            return q_tmp;
        }

        void Robot::test(){
            cout << "robot tested" << endl;
        }
        void Robot::set_msg(string s){
            //msg.data = s;
        }
        string Robot::get_msg(){
            return msg.data;
        }
        int Robot::print_position(sensor_msgs::JointState j){
            
            //Robot r;
            //cout << j << endl;

            if(!Robot::started){
                cout << "nessun messaggio joint" << endl;
                return -1;
            }

            //cout << "printing joints" << endl;

            for(int i=0;    i< 6;i++){
                cout << "posizione " << i << ": " << j.position[i] << endl;
            }
            //cout << "done printing" << endl;
            cout << "--------------------------------------------------" << endl << endl;
            return 1;
        };
        int Robot::print_position(){
            return print_position(joint);
        };

//std_msgs::String Robot::msg = new std_msgs::String();
void robot_msg_callback(const std_msgs::String::ConstPtr& message){
    //cout << "in msg_callback" << endl;
    //ROS_INFO("I heard: [%f]", js->velocity.c_str());
    //ROS_INFO("msg");
	//ROS_INFO("I heard: [%s]", message->data.c_str());
    //cout << "msg: " << message->data.c_str() << endl;
    cout << "---------------------------------------------" << endl;
    Robot::stat++;
    //cout << "stat: " << Robot::stat << endl;
	//js=joint;

    //Robot robot;
    //robot.test();
    //robot.set_msg(message->data.c_str());
    //robot.msg.data = message->data.c_str();

    //std_msgs::String m;
    //m.data = message->data.c_str();
    //std_msgs::String Robot::msg = m;
}
/*! @brief callback for ur5/joint_states \n
* detects the number of joints, saves the eventual gripper values, \n
* reorders the joints values and saves them
*/

void js_callback(const sensor_msgs::JointState& j){

    //ROS_INFO("I heard: [%f]", j.velocity.c_str());
    //ROS_INFO("js");
	/*
    cout << "vel0: " << j.position[0]<< endl;
    cout << "vel1: " << j.position[1]<< endl;
    cout << "vel2: " << j.position[2]<< endl;
    cout << "vel3: " << j.position[3]<< endl;
    cout << "vel4: " << j.position[4]<< endl;
    cout << "vel5: " << j.position[5]<< endl;
    cout << "---------------------------------------------";
	//js=joint;
    */
    Robot::joint = sensor_msgs::JointState(j);//
    Robot::n_grip=Robot::joint.position.size()-6;//6 per joint il resto per gripper

    Robot::gripper_j.clear();

    for(int i=0; i < Robot::n_grip; i++){

        Robot::gripper_j.push_back( Robot::joint.position[1] );   //da posizione 1 
        Robot::joint.position.erase(std::next(Robot::joint.position.begin(), 1)); //toglie il secondo elemento n volte

    }
    
    swap(Robot::joint.position[0],Robot::joint.position[2]); //perche si
    Robot::started = true;
    
    //print_position();
}

ostream& operator<<(ostream& os, const my_vision_messages::Pointxyz& p){
        os << "x: " << p.x << "\ty: " << p.y << "\tz: " << p.z;
        return os;
}

ostream& operator<<(ostream& os, const my_vision_messages::BlockList& bl){
    if(bl.blocks.size()<1){
        os<< "BlockList vuota";
        return os;
    }
    for(int i= 0; i< bl.blocks.size(); i++){
        os << "class number: " << bl.blocks[i].class_number << "\n";
        os << "point camera: " << bl.blocks[i].point << "\n";
        os << "rotation angle: " << bl.blocks[i].rot_angle << "\n";
        for(int j=0; j< bl.blocks[i].top_btm_l_r.size(); j++){
            os << "\t" << bl.blocks[i].top_btm_l_r[j] << "\n";
        }
            os << "up_dw_stand_lean: " << bl.blocks[i].up_dw_stand_lean << "\n";
            os << "confidance: " << bl.blocks[i].confidence << "\n";
            os << "world frame point: " << bl.blocks[i].world_point << "\n";
    }
    return os;
}

bool operator==( const my_vision_messages::BlockList& bl1, const my_vision_messages::BlockList& bl2 ){
    if(bl1.blocks.size()!=bl2.blocks.size() || bl1.blocks.size()==0 || bl2.blocks.size()==0)
        return false;
    for(int i=0; i< bl1.blocks.size(); i++){
        if(bl1.blocks[i].class_number == bl2.blocks[i].class_number
        && bl1.blocks[i].world_point.x - bl2.blocks[i].world_point.x < BL_DIFF_RANGE
        && bl1.blocks[i].world_point.y - bl2.blocks[i].world_point.y < BL_DIFF_RANGE
        && bl1.blocks[i].world_point.z - bl2.blocks[i].world_point.z < BL_DIFF_RANGE
        && bl1.blocks[i].confidence - bl2.blocks[i].confidence < BL_DIFF_CONFIDANCE)
        //ok
            int a=0;
        else return false;
    }
    return true;
    
}

void print_BlockList(my_vision_messages::BlockList& bl){
    cout << bl;
}

void swap(double& f1,double& f2){
    float buff;
    buff=f1;
    f1=f2;
    f2=buff;
}


#endif
