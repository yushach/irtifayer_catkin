
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

#include <std_msgs/Float64.h>

#include <sensor_msgs/Imu.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <cstdlib>
#include <time.h>
#include <ctime>

#define info ROS_INFO_STREAM
#define error ROS_ERROR_STREAM
#define warn ROS_WARN_STREAM

#include "opencv2/core/core.hpp"


using namespace std;
using namespace cv;

#define move_lim 10


class PlanPoint{
    public:
    float lat;
    float lng;
    float alt;
    float vel;
    std::string role;
};


double lastlat = 41.08;
double lastlng = 29.08;

std::string ihacomm = "/tmp/ihacomm";
std::string ihatelem = "/tmp/ihatelem";
std::string wpbildirim = "/tmp/wpbildirim";


clock_t start;
time_t starttime, endtime;


const double ALT_ADD = 10;
const double DEF_ALT = 560.0; //altitude of that airport in australia

double findlat = -35.3629204;
double findlng = 149.1649819; //will be overriden later for randomness
double waterlat = -99; double waterlng = -99; //will be set to wherever we pretend the water will be

double global_yaw;
double global_vx;
double global_vy;
double global_vz;

int loopcounter = 0;

int gobackuntil = 0;
bool stoppedforwater = false;
int afterwaterresumeat;
int waterdiscoverlap = -1; //to make sure we stop at the lap after we discover the water

//MARK REGION function declarations
void tf_callback(const nav_msgs::Odometry::ConstPtr& msg);
void gps_callback(const mavros_msgs::GPSRAW::ConstPtr& msg);
mavros_msgs::Waypoint createWp(double setlat, double setlng=149.1653466, double setalt=4.0);
int set_mode(std::string mode);
void clearMission(); void setHome();
void update_telem_time();

void send_waypoints(std::vector<PlanPoint> plan);
void write_updated_waypoints(std::vector<PlanPoint> plan);
std::vector<PlanPoint> modify_plan(std::vector<PlanPoint> plan);
void print_plan(std::vector<PlanPoint> plan);
int get_index(std::vector<std::string> vector, std::string key);
void add_to_telemetry(std::string key, std::string val);
void process_telem(std::vector<std::string> strlist);
void get_telem();
void write_telem();
//ENDREGION

std::vector<std::string> telemkeys;
std::vector<std::string> telemvals;
std::vector<std::string> telem;

double pub_x, pub_y; //x and y speeds to publish relatively
double cur_gps_lat, cur_gps_lng, cur_alt; //current coordinates

ros::Publisher pub_wpl; //waypoint publisher, so far useless

bool cansetwp = false;
bool neversetwpagain = false; //to only initialize wp list once

int seq; //rostopic'ten current_seq
int reached; //son varılan rota noktası
int prevreached; //sondan önceki rota noktası
int wpcount; //rota noktası sayısı

//varsayılan waypointler
/*mavros_msgs::Waypoint wp0 = createWp(-35.3630587, 149.1650006, DEF_ALT+ALT_ADD);
mavros_msgs::Waypoint wp1 = createWp(-35.3626168, 149.1649483, DEF_ALT+ALT_ADD);
mavros_msgs::Waypoint wp2 = createWp(-35.3624856, 149.1651428, DEF_ALT+ALT_ADD);
mavros_msgs::Waypoint wp3 = createWp(-35.3625807, 149.1653171, DEF_ALT+ALT_ADD);
mavros_msgs::Waypoint wp4 = createWp(-35.3629865, 149.1653909, DEF_ALT+ALT_ADD);
mavros_msgs::Waypoint wp5 = createWp(-35.3631188, 149.1652058, DEF_ALT+ALT_ADD);*/
std::vector<mavros_msgs::Waypoint> wplist;

ros::ServiceClient set_mode_client;
ros::ServiceClient wp_clear_client;
ros::ServiceClient wp_srv_client;
ros::ServiceClient set_home_client;
mavros_msgs::WaypointPush wp_push_srv;
mavros_msgs::WaypointClear wp_clear_srv;
mavros_msgs::CommandHome set_home_srv;

ros::ServiceClient arm_client;
mavros_msgs::CommandBool arm_srv;
ros::ServiceClient takeoff_client;
mavros_msgs::CommandTOL takeoff_srv;

double speed = 1;

bool wrote_telem = false;

//başlama komutu ver
void start_lap(){
    set_mode("GUIDED");
    wpcount = wplist.size();
    set_mode("AUTO");
}

//görevi sıfırla
void clearMission(){
    wp_clear_srv.request = {};
    if (wp_clear_client.call(wp_clear_srv)){
        info("Gorev sifirlama istegi gonderildi");
    }else{
        error("Gorev sifirlama basarisiz");
    }
}
//kalkış noktasını home olarak ayarla (kalkış yerine göre değiştir)
void setHome(){
    set_home_srv.request.current_gps = false;
    set_home_srv.request.latitude = 41.3632695;
    set_home_srv.request.longitude = 29.1652326;
    set_home_srv.request.altitude = DEF_ALT; //change to 0 later

    if (set_home_client.call(set_home_srv)){
        info("home ayarlama istegi gonderildi");
    }else{
        error("home ayarlanamadi");
    }
}

//rota noktaları listesini güncelle
void setWpList(){
    wp_push_srv.request.waypoints.clear();
    for (int i=0; i<wplist.size(); i++){
        wp_push_srv.request.waypoints.push_back(wplist[i]);
    }
    if (wp_srv_client.call(wp_push_srv)){
         info("rota noktalari gonderildi");
    };
}

//mavros_msgs/Waypoint formatından kendi nokta formatımıza dönüştür
std::vector<PlanPoint> ppfromwpl(std::vector<mavros_msgs::Waypoint> thelist){
    std::vector<PlanPoint> newlist;
    //birincisi 2 kez eklenmiş oluyor o yüzden indeksi 1'den başlatıyoruz
    for (int i=1; i<thelist.size(); i++){
        PlanPoint newpoint;
        newpoint.lat = thelist[i].x_lat;
        newpoint.lng = thelist[i].y_long;
        newpoint.alt = thelist[i].z_alt;
        switch (thelist[i].command){
            case 16:
                newpoint.role = "NORMAL"; break;
            case 22:
                newpoint.role = "NORMAL"; break;
            case 21:
                newpoint.role = "NORMAL"; break; //iniş, kalkış veya waypoint
            default:
                newpoint.role = "POI"; break; //ileride kullanılabilir
        }
        newlist.push_back(newpoint);
    }
    //std::cout << std::setiosflags(ios::fixed) << std::setprecision(10) << newlist[0].lat << std::endl;
    return newlist;
}

void wp_callback(const mavros_msgs::WaypointList::ConstPtr& msg){
    //info("### WPList ###");
    seq = msg->current_seq;
    wpcount = msg->waypoints.size();
    warn("guncel rota noktalari alindi:" << wpcount);
    write_updated_waypoints(ppfromwpl(msg->waypoints));
}

mavros_msgs::Waypoint createWp(double setlat, double setlng, double setalt){
    mavros_msgs::Waypoint wp;
    wp.x_lat = setlat; wp.y_long = setlng; wp.z_alt = setalt; //önemli parametreler
    wp.param1 = 0.0; wp.param2 = 0.0; wp.param3 = 0.0; wp.param4 = 0.0; //varsayılan değerler
    wp.frame = 3; //relative
    wp.command = 16;
    wp.autocontinue = true;
    //is_current değişkenini olduğu gibi bırakıyorum
    return wp;
}

mavros_msgs::Waypoint makeCurrent(mavros_msgs::Waypoint wp){
    wp.is_current = true;
    return wp;
}

mavros_msgs::Waypoint makeTakeoff(mavros_msgs::Waypoint wp){
    wp.command = 22;
    return wp;
}

mavros_msgs::Waypoint makeLand(mavros_msgs::Waypoint wp){
    wp.command = 21;
    return wp;
}


//debug içindi
/*void do_random_gps(){
    srand((unsigned) time(0));
    double latinc = (rand() % 4) - 2;
    double lnginc = (rand() % 4) - 2;
    double sendlat = lastlat + latinc;
    double sendlng = lastlng + lnginc;
    add_to_telemetry("LAT", std::to_string(sendlat));
    add_to_telemetry("LNG", std::to_string(sendlng));
    wrote_telem = false;
    lastlat = sendlat;
    lastlng = sendlng;
}*/

//callback ile biten fonksiyonlar ilgili rostopic subscriberlarına bağlı
void gps_callback(const mavros_msgs::GPSRAW::ConstPtr& msg){
    //info("### GPS ###");
    int latint = msg->lat;
    int lngint = msg->lon;
    int altint = msg->alt;
    cur_gps_lat = ((double) latint) * pow(10, -7);
    cur_gps_lng = ((double) lngint) * pow(10, -7);
    cur_alt = ((double) altint) * 0.001;
    cansetwp = true;

    add_to_telemetry("LAT", std::to_string(cur_gps_lat));
    add_to_telemetry("LNG", std::to_string(cur_gps_lng));
    wrote_telem = false;
}

void alt_callback(const std_msgs::Float64::ConstPtr& msg){
    //info("ALT: " << msg->data);
    add_to_telemetry("ALT", std::to_string(msg->data));
    wrote_telem = false;
}

//görevi temizleyip rotayı güncelle
void updateWaypoints(){
    set_mode("GUIDED");
    clearMission();
    setWpList();
}

void reached_callback(const mavros_msgs::WaypointReached::ConstPtr& msg) {
    //sadece iha üzerindeki yazılımda kullanılacak
}

void tf_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry odom = *msg;
    geometry_msgs::Quaternion orient = odom.pose.pose.orientation;
    tf::Quaternion q(
        orient.x,
        orient.y,
        orient.z,
        orient.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //info("YAW: " << yaw);
    global_yaw = yaw;
    pub_x = speed * cos(yaw);
    pub_y = speed * sin(yaw);
    //double airspeed = std::sqrt(pub_x*pub_x + pub_y*pub_y);
    add_to_telemetry("HDNG", std::to_string(yaw));
    add_to_telemetry("ATTP", std::to_string(pitch));
    add_to_telemetry("ATTR", std::to_string(roll));
    //add_to_telemetry("AIRS", std::to_string(airspeed));
    wrote_telem = false;
}

int set_mode(std::string mode) //bu fonksiyonu iq_gnc paketinden aldım, mod değiştirmek için
{
	mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = mode.c_str();
    if(set_mode_client.call(srv_setMode)){
      info("mod degistirme istegi gonderildi");
	  return 0;
    }else{
      error("mod degistirme istegi gonderilemedi");
      return -1;
    }
}

double radtodeg(double rad){
    return rad*180/3.141592653589793238;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    geometry_msgs::Quaternion orient = msg->orientation;
    tf::Quaternion q(
        orient.x,
        orient.y,
        orient.z,
        orient.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //ROS_INFO_STREAM("YAW: " << yaw);
    global_yaw = yaw;
    pub_x = speed * cos(yaw);
    pub_y = speed * sin(yaw);

    yaw = radtodeg(yaw);
    pitch = radtodeg(pitch);
    roll = radtodeg(roll);

    add_to_telemetry("HDNG", std::to_string(yaw));
    add_to_telemetry("ATTP", std::to_string(pitch));
    add_to_telemetry("ATTR", std::to_string(roll));
    wrote_telem = false;

    //add_to_telemetry("AIRS", std::to_string(airspeed));
}

void state_callback(const mavros_msgs::State::ConstPtr& msg){
    bool armed = msg->armed;
    std::string mode = msg->mode;
    std::string armtext = armed ? "true" : "false";
    add_to_telemetry("ARMED", armtext);
    add_to_telemetry("MODE", mode);
    wrote_telem = false;
}

//**********************************************************************************************************************

int arm(){
    //arm
    info("arm ediliyor");
    arm_srv.request.value = true;
    bool do_arm = arm_client.call(arm_srv);

    if (do_arm){
        info("arm istegi gonderildi");
        
        return 0;
    };
    error("arm istegi gonderilemedi");
    return 1;
}

int disarm(){
    //disarm
    info("disarm ediliyor");
    arm_srv.request.value = false;
    bool do_disarm = arm_client.call(arm_srv);
    if (do_disarm){
        info("disarm istegi gonderildi");
        
        return 0;
    };
    error("disarm istegi gonderilemedi");
    return 1;
}


int takeoff(double targetalt){
    //kalkış
    info("Kalkis yapiliyor");
    takeoff_srv.request.yaw = global_yaw;
    takeoff_srv.request.latitude = cur_gps_lat;
    takeoff_srv.request.longitude = cur_gps_lng;
    takeoff_srv.request.altitude = targetalt;
    if (takeoff_client.call(takeoff_srv)){
        info("kalkis istegi gonderildi");
        
        return 0;
    };
    error("kalkis istegi gonderilemedi");
    return 1;
}

//rosservice aracılığıyla rota noktalarını gönder
void send_waypoints(std::vector<PlanPoint> plan){
    
    wp_push_srv.request.waypoints.clear();
    for (int i=0; i<plan.size(); i++){
        mavros_msgs::Waypoint wp = createWp(plan[i].lat, plan[i].lng, plan[i].alt); //DEF_ALT düzenleyip ekle!
        if (i==0){
            wp = makeCurrent(wp);
            wp_push_srv.request.waypoints.push_back(wp);
        }
        wp_push_srv.request.waypoints.push_back(wp);
        
    }
    
    if (wp_srv_client.call(wp_push_srv)){
         info("Rota noktalari gonderildi");
    }else{
        error("Rota noktalari gonderilemedi");
    }
}

//güncel rota noktalarını dosya sistemine kaydet
void write_updated_waypoints(std::vector<PlanPoint> plan){
    std::ofstream ofs;
    ofs.open(wpbildirim, std::ofstream::out | std::ofstream::trunc);
    for (int i=0; i<plan.size(); i++){
        ofs << std::setiosflags(ios::fixed) << std::setprecision(15)
        << plan[i].lat << ","
        << plan[i].lng << ","
        << plan[i].alt << ","
        << plan[i].vel << ","
        << plan[i].role
        << "\n";
    }
    ofs << "END\n";
    info("Plan diske yazildi");
    ofs.close();
}

//debug içindi
/*std::vector<PlanPoint> modify_plan(std::vector<PlanPoint> plan){
    PlanPoint randp = plan[1]; randp.lat += 0.05; randp.lng -= 0.05;
    plan.insert(plan.begin()+1, randp);
    return plan;
}*/

//debug için, rotayı basıyor
void print_plan(std::vector<PlanPoint> plan){
    info(" *** Rota ***");
    for (int i=0; i<plan.size(); i++){
        std::cout << "P" << i
        << " LAT: " << plan[i].lat
        << " LNG: " << plan[i].lng
        << " ALT: " << plan[i].alt
        << " AIRS: " << plan[i].vel
        << std::endl;
    }
    info("");
}

//telemetri listesinde hazırda bulunan keylerin indeksini almak için
int get_index(std::vector<std::string> vector, std::string key){
    for (int i=0; i<vector.size(); i++){
        if (vector[i].compare(key) == 0){
            return i;
        }
    }
    return -1;
}

//telemetri listesine eleman ekle veya varolan elemanı güncelle
void add_to_telemetry(std::string key, std::string val){
    //info("Adding to telemetry");
    std::string fullword = key + ":" + val;
    if (std::count(telemkeys.begin(), telemkeys.end(), key)) {
        int index = get_index(telemkeys, key);
        telem[index] = fullword;
    }else{
        //std::string fullword = key + ":" + val;
        telemkeys.push_back(key);
        telemvals.push_back(val);
        telem.push_back(fullword);
    }
}

void process_telem(std::vector<std::string> strlist){
    // alınan komutları işle
    try{
        if (strlist.size() > 0){
            // son satır END ise komutu işle
            if (strlist.back().compare("END") == 0){
                if (strlist[0].compare("CMD_ARM") == 0){
                    arm();
                }
                else if (strlist[0].compare("CMD_DISARM") == 0){
                    disarm();
                }
                else if (strlist[0].compare("CMD_START") == 0){
                    start_lap();
                }
                else if (strlist[0].compare("CMD_MODE") == 0){
                    set_mode(strlist[1]);
                }
                else if (strlist[0].compare("CMD_TAKEOFF") == 0){
                    takeoff(stod(strlist[1]));
                }
                // rota noktaları paylaşıldı
                else if (strlist[0].compare("WPS") == 0){
                    info("Rota noktalari alindi");
                    std::vector<PlanPoint> newplan;
                    for(int i=1; i<strlist.size()-1; i++){
                        //std::cout << strlist[i] << std::endl;
                        PlanPoint newpt;
                        std::vector<std::string> prms;

                        std::string segment;
                        std::stringstream stream(strlist[i]);
                        while(std::getline(stream, segment, ','))
                        {
                            prms.push_back(segment);
                        }
                        newpt.lat = atof(prms[0].c_str());
                        newpt.lng = atof(prms[1].c_str());
                        newpt.alt = atof(prms[2].c_str());
                        newpt.vel = atof(prms[3].c_str());
                        //std::cout << prms[4] << std::endl;
                        newpt.role = prms[4];
                        newplan.push_back(newpt);
                    }
                    print_plan(newplan);
                
                    send_waypoints(newplan);
                }
                
            }else{
                //komut yok
            }
        }
    }catch (int number){}
}

void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    global_vx = msg->twist.linear.x;
    global_vy = msg->twist.linear.y;
    global_vz = msg->twist.linear.z;
    add_to_telemetry("VRTS", std::to_string(global_vz));
    add_to_telemetry("AIRS", std::to_string(sqrt(global_vx*global_vx + global_vy*global_vy)));
    wrote_telem = false;
}

void get_telem(){
    // yer istasyonundan gelen verileri oku
    std::ifstream f(ihacomm);
    std::string line;
    std::vector<std::string> strlist;
    while ( std::getline (f,line) )
    {
        //std::cout << line << std::endl;
        strlist.push_back(line);
    }
    f.clear();
    f.seekg(0,std::ios::beg);
    f.close();

    // verileri işle
    process_telem(strlist);

    // iletişim dosyasını temizle
    std::ofstream ofs;
    ofs.open(ihacomm, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
}

void write_telem(){
    if (!wrote_telem){
        update_telem_time();
        std::ofstream ofs;
        ofs.open(ihatelem, std::ofstream::out | std::ofstream::trunc);
        for (int i=0; i<telem.size(); i++){
            ofs << telem[i] << "\n";
        }
        ofs << "END\n";
        
        ofs.close();
        wrote_telem = true;
    }
}

//telemetriye zaman verisini ekle
void update_telem_time(){
    time(&endtime);
    add_to_telemetry("LAST", std::to_string((endtime - starttime)));
}

//************************************************************************************************************************* DA MAIN LOOP
int main(int argc, char** argv)
{
    time(&starttime);

    srand((unsigned) time(0)); //initialize random seed
    

	ros::init(argc, argv, "ihavekil");
    ros::NodeHandle nh;

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    wp_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
    wp_srv_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
    set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    arm_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    pub_wpl = nh.advertise < mavros_msgs :: WaypointList >("/mavros/mission/waypoints", 1000);
    
    ros::Subscriber sub_gps = nh.subscribe("/mavros/gpsstatus/gps1/raw", 1000, gps_callback);
    ros::Subscriber sub_wpl = nh.subscribe("/mavros/mission/waypoints", 1000, wp_callback);
    ros::Subscriber sub_reached = nh.subscribe("/mavros/mission/reached", 1000, reached_callback);
    ros::Subscriber sub_vel = nh.subscribe("/mavros/global_position/raw/gps_vel", 1000, vel_callback);

    ros::Subscriber sub_alt = nh.subscribe("/mavros/global_position/rel_alt", 1000, alt_callback);
    ros::Subscriber sub_imu = nh.subscribe("/mavros/imu/data", 1000, imu_callback);
    ros::Subscriber sub_state = nh.subscribe("/mavros/state", 1000, state_callback);


    setHome();
    ros::Rate loop_rate(10);
    bool wasop = false;

    info("Ana donguye baslaniyor");
    while (ros::ok()){
        //info("LOOP:" << loopcounter);
        get_telem();
        write_telem();

        ros::spinOnce();
        loop_rate.sleep();
        loopcounter++;
    }
	return 0;
}


