#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>


using namespace gtsam;



struct GpsMeasurement {
  double time;
  Vector3 position;  // x,y,z

};
// class UnaryFactor: public NoiseModelFactor1<Pose2> {
//   double mx_, my_; ///< X and Y measurements

// public:
//   UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
//     NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

//   Vector evaluateError(const Pose2& q,
//                        boost::optional<Matrix&> H = boost::none) const
//   {
//     const Rot2& R = q.rotation();
//     if (H) (*H) = (gtsam::Matrix(2, 3) <<
//             R.c(), -R.s(), 0.0,
//             R.s(), R.c(), 0.0).finished();
//     return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
//   }


// };

// Agents - each agent creates an array of other agents
// to stay aware of where the other agents are

class Agent {

    private:
        Pose3 pose_world_noisy;
        std::vector<double> x_tr;
        std::vector<double> y_tr;
        std::vector<double> z_tr;
        std::vector<double> x_noisy;
        std::vector<double> y_noisy;
        std::vector<double> z_noisy;
        std::vector<double> roll_tr;
        std::vector<double> pitch_tr;
        std::vector<double> yaw_tr;
        int step;
        int agentNumber;
        Pose3 prev_pose_world_noisy_comms;
        int poseKey;
        int prevPoseKey;
        int poseKeyStart;
    
    public:

        Agent(){}
        Agent(Pose3 H_init, int agentNumber){

            // std::cout << "agent number cpp: " << agent_number << std::endl;
            pose_world_noisy = H_init;
            prev_pose_world_noisy_comms = H_init;
            poseKey = (agentNumber + 1) * 1e6;
            prevPoseKey = poseKey;
            poseKeyStart = poseKey;
            agentNumber = agentNumber;

        }
        ~Agent(){}


        void SetXNoisy(std::vector<double> x){
            x_noisy = x;
        }
        std::vector<double> GetXNoisy(){
            return x_noisy;
        }
        void PushBackXNoisy(double value){
            x_noisy.push_back(value);
        }

        void SetYNoisy(std::vector<double> x){
            y_noisy = x;
        }
        std::vector<double> GetYNoisy(){
            return y_noisy;
        }
        void PushBackYNoisy(double value){
            y_noisy.push_back(value);
        }

        void SetXTrue(std::vector<double> x){
            x_tr = x;
        }
        std::vector<double> GetXTrue(){
            return x_tr;
        }
        void PushBackXTrue(double value){
            x_tr.push_back(value);
        }
        void SetYTrue(std::vector<double> y){
            y_tr = y;   
        }
        std::vector<double> GetYTrue(){
            return y_tr;
        }
        void PushBackYTrue(double value){
            y_tr.push_back(value);
        }
        void SetZTrue(std::vector<double> z){
            z_tr = z;
        }
        std::vector<double> GetZTrue(){
            return z_tr;
        }
        void PushBackZTrue(double value){
            z_tr.push_back(value);
        }
        void SetRollTrue(std::vector<double> r){
            roll_tr = r;
        }

        std::vector<double> GetRollTrue(){
            return roll_tr;
        }

        void SetPitchTrue(std::vector<double> p){
            pitch_tr = p;
        }
        std::vector<double> GetPitchTrue(){
            return pitch_tr;
        }
        void SetYawTrue(std::vector<double> y){
            yaw_tr = y;
        }
        std::vector<double> GetYawTrue(){
            return yaw_tr;
        }

        void updateStep(int nextStep){
            step = nextStep;
        }

        void UpdatePoseKey(){
            prevPoseKey = poseKey;
            poseKey++;
        }

        int getStep(){
            return step;
        }
        int getAgentNumber(){
            return agentNumber;
        }
        Pose3 GetPoseWorldNoisy(){
            return pose_world_noisy;
        }
        Pose3 getPrevWorldNoisyComms(){
            return prev_pose_world_noisy_comms;
        }
        void UpdatePrevWorldNoisyComms(){
            prev_pose_world_noisy_comms = GetPoseWorldNoisy();

        }
        int getPoseKey(){
            return poseKey;
        }
        int getPrevPoseKey(){
            return prevPoseKey;
        }
        int getPoseKeyStart(){
            return poseKeyStart;
        }
        void SetPoseWorldNoisy(Pose3 new_pose){
            pose_world_noisy = new_pose;
        }
};

class FactorStateNode : public rclcpp::Node
{
public:
    FactorStateNode()
        : Node("factor_state_node")
    {

        // gtsam stuff
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        parameters.cacheLinearizedFactors = false;
        parameters.enableDetailedResults = true;
        isam = ISAM2(parameters);
        sigmas << Vector3::Constant(0.1), Vector3::Constant(0.008);
        odometryNoise = noiseModel::Diagonal::Sigmas(sigmas);
        noise_model_gps << Vector3::Constant(0), Vector3::Constant(1.0 / 0.07)
        priorFactorInit = true;

        unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y

        // Set up the subscriptions
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/modem_imu", 10,
            std::bind(&FactorStateNode::imuCallback, this, std::placeholders::_1));

        gps_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/gps/odom", 10,
            std::bind(&FactorStateNode::gpsOdomCallback, this, std::placeholders::_1));

        dvl_dead_reckon_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dvl_dead_reckon", 10,
            std::bind(&FactorStateNode::dvlDeadReckonCallback, this, std::placeholders::_1));

        // Set up the publisher
        factor_state_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/factor_state", 10);
    }

private:
    
    // this is the agent object to
    // keep track of where it is at
    Agent * agent;
    double theta;

    // gtsam stuff
    NonlinearFactorGraph graph;
    Values initialEstimate;
    ISAM2 isam;
    Values result;
    Vector6 std_noise;
    Vector6 sigmas;
    Vector6 noise_model_gps;
    noiseModel::Diagonal::shared_ptr odometryNoise;
    bool priorFactorInit;
    noiseModel::Diagonal::shared_ptr gpsNoise;

    bool initial_rotation = false
    Eigen::Matrix3d R
    Eigen::Matrix2d R2D 

    void update_est() {
        isam.update(graph,initialEstimate);
        result = isam.calculateEstimate();
        graph.resize(0);
        initialEstimate.clear();
    }

    Pose3 CalculateH(const nav_msgs::msg::Odometry & msg){

            Eigen::Quaterniond q;

    
            q.x() = msg.pose.pose.orientation.x;
            q.y() = msg.pose.pose.orientation.y;
            q.z() = msg.pose.pose.orientation.z;
            q.w() = msg.pose.pose.orientation.w;
            

            Eigen::Matrix3d R = q.toRotationMatrix();

            Eigen::Matrix4d H;

            H.block(0, 0, 3, 3) = R;
            

            H(0,3) = msg.pose.pose.position.x;
            H(1,3) = msg.pose.pose.position.y;
            H(2,3) = msg.pose.pose.position.z;
            

            H(3,0) = 0;
            H(3,1) = 0;
            H(3,2) = 0;
            H(3,3) = 1;

            return Pose3(H);

        }


   
    // Callback functions for each topic
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {


        // w = msg->orientation.w
        // x = msg->orientation.x
        // y = msg->orientation.y 
        // z = msg->orientation.z

        if(!initial_rotation) {
            // Create a quaternion from the message orientation
            Eigen::Quaterniond q;

            q.x() = msg->orientation.x;
            q.y() = msg->orientation.y;
            q.z() = msg->orientation.z;
            q.w() = msg->orientation.w;
            
            // Convert quaternion to 3x3 rotation matrix
            R = q.toRotationMatrix();

            // Extract the upper-left 2x2 submatrix for 2D rotation
            // R2D = R.block<2, 2>(0, 0);

            // Mark that the initial rotation has been set
            initial_rotation = true;
        }
        


        // // Process the IMU data here
        // RCLCPP_INFO(this->get_logger(), "Received IMU data.");
        // // Example: Print the IMU orientation
        // RCLCPP_INFO(this->get_logger(), "Orientation x: %f, y: %f, z: %f, w: %f",
        //             msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

        // Integrate data processing logic here
        publishFactorState();
    }

    void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr& msg)
    {   

        GpsMeasurement measurement = Vector3(msg.pose.pose.position.x, msg.pose.pose.position.x, 0.0);

        // Process the GPS odometry data here
        RCLCPP_INFO(this->get_logger(), "Received GPS odometry data.");
        // Example: Print the position
        RCLCPP_INFO(this->get_logger(), "Position x: %f, y: %f",
                    msg.pose.pose.position.x, msg.pose.pose.position.y);

        if(priorFactorInit){
            Pose3 H_init = CalculateH(msg);
            agent = new Agent(H_init, msg.agent_number);
            priorFactorInit = false;
            graph.push_back(PriorFactor<Pose3>(agent->getPoseKey(),agents->GetPoseWorldNoisy(), odometryNoise));
            initialEstimate.insert(agents->getPoseKey(), agents->GetPoseWorldNoisy());
        }
        else{
            auto gps_pose = Pose3(agent->GetPoseWorldNoisy().rotation(), measurement.position);
            graph.push_back(PriorFactor<Pose3>(agent->getPoseKey(), gps_pose, noise_model_gps));
            initialEstimate.insert(agents->getPoseKey(), gps_pose);
        }

        // Integrate data processing logic here
        publishFactorState();
    }

    void dvlDeadReckonCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        // Process the DVL dead reckoning data here
        RCLCPP_INFO(this->get_logger(), "Received DVL dead reckoning data.");
        // Example: Print the velocity
        RCLCPP_INFO(this->get_logger(), "Position x: %f, y: %f",
                    msg->pose.pose.position.x, msg->pose.pose.position.y);



        if(!priorFactorInit){

            /// TODO: convert dvl x, y to local x,y grid coordinates
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;

            Eigen::Vector2d v(x,y);
            Eigen::Matrix2d mat;

            // review this , possibly use the transpose
            mat << R2D(0,0), R2D(0,1),
                   R2D(1,0), R2D(1,1);

            Eigen::Vector2d v_g = mat * v;


            // do transformation right here
            Pose3 H_pose2_wrt_pose1_noisy = CalculateH(msg);
            Pose3 poseWorldNoisy = CalculateH(msg);
            agent->SetPoseWorldNoisy(poseWorldNoisy);
            agent->UpdatePoseKey();
            initialEstimate.insert(agent->getPoseKey(), agent->GetPoseWorldNoisy());
            graph.add(BetweenFactor<Pose3>(agent->getPrevPoseKey(),agent->getPoseKey(),H_pose2_wrt_pose1_noisy,odometryNoise));



        }

        // Integrate data processing logic here
        
        publishFactorState();
    }

    void publishFactorState()
    {
        update_est();
        // Create and publish the /factor_state message
        auto factor_state_msg = nav_msgs::msg::Odometry();
        // Populate the factor_state_msg here with the processed data
        factor_state_msg->pose.pose.position.x = 
        factor_state_msg->pose.pose.position.y = 
        
        factor_state_publisher_->publish(factor_state_msg);
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_dead_reckon_subscriber_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr factor_state_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FactorStateNode>());
    rclcpp::shutdown();
    return 0;
}
