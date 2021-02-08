#ifndef _NCAP_SERVER_
#define _NCAP_SERVER_


#include <zmq.hpp>
#include "oppt/problemEnvironment/ProblemEnvironment.hpp"
#include "NCAPCollisionObservation.hpp"
#include "../ABT/solverABT.hpp"
#include "../ABT/ABTOptions.hpp"
#include <string>
#include <iostream>
#include "json.hpp"


using json = nlohmann::json;


namespace oppt {

class NcapServer{

private: 
    // Member variables
    // context for inter process communication
    zmq::context_t ctx_;
    zmq::socket_t ipc_sock_;
    std::unique_ptr<ProblemEnvironment> problemEnvironment_ = nullptr;
    ActionSharedPtr lastAction_ = nullptr;
    solvers::Solver* solver_ = nullptr;
    FloatType planningTime_;

public:
    
    NcapServer(FloatType planTime): 
        ctx_{1}, ipc_sock_{ctx_, zmq::socket_type::rep}, planningTime_{planTime} {
        // Initialize problem environment
        problemEnvironment_ = std::make_unique<oppt::ProblemEnvironment>();
    }


    ~NcapServer(){}


    /*** Sets up the problem environment according to the config file provided ***/
    bool setupOpptEnvironment(int argc, char const* argv[]){
        std::cout << "INPUT WAS" << std::endl;
        std::cout << argv << std::endl;

        int ret = problemEnvironment_->setup<solvers::ABT, oppt::ABTExtendedOptions>(argc, argv);
        if(ret != 0){
            return false;
        }


        // With a problem environment correctly setted up, set up the solver
        solver_ = problemEnvironment_->getSolver();
        if(!solver_->reset()){
            std::cout << "COULD NOT RESET SOLVER" << std::endl;
            return false;
        }

        // No problem return true for success
        return true; 
    }


    /*** Start the inter process communication binding to ipc socket ***/
    void bindServer(const std::string ipc_path){
        // // Test the communication step here
        std::cout << "BINDING SERVER TO " + ipc_path << std::endl;
        ipc_sock_.bind(ipc_path); 
    }



   

    /*** Function to update the in built viewer of oppt ***/
    bool updateViewer(){
        auto particles = static_cast<solvers::ABT *>(problemEnvironment_->getSolver())->getBeliefParticles();
        auto currentState = particles[0];
        problemEnvironment_->updateViewer(currentState,particles);
    }






    /*** Waits for a message request and executes the appropiate server function for the service requests ***/
    void parseMessage(){
         // receive reply
        zmq::message_t zmq_msg;
        std::cout << "RECEIVING MESSAGE" << std::endl;

        ipc_sock_.recv(zmq_msg);
        const std::string recv_msg(zmq_msg.data<char>(), zmq_msg.size());
        std::cout << "MESSAGE RECEIVED WAS\n" <<  recv_msg << std::endl;

        // Convert back to JSON
        json json_struct = json::parse(recv_msg);
        

        // Server request according to type
        if( compareStrings(json_struct["TYPE"], "RESET") ){
            std::cout << "RESTART QUERY RECEIVED" << std::endl;
            resetSolver(json_struct);
        } else if( compareStrings(json_struct["TYPE"], "NEXT_ACTION") ){
            std::cout << "ACTION QUERY RECEIVED" << std::endl;
            sendNextAction(json_struct);
        } else if( compareStrings(json_struct["TYPE"], "UPDATE_BELIEF") ){
            std::cout << "BELIEF UPDATE RECEIVED" << std::endl;
            updateCurrentBelief(json_struct);
        }
    }



private:
    // Helper functions

     /*** Resets the solver to start problem from scratch ***/
    void resetSolver(json json_req){
        // Modify local copy of the json struct to send reply message
        json_req["TYPE"] = "RESET_ACK";
        // Attempt to reset the solver
        if(!solver_->reset()){
            std::cout << "COULD NOT RESET SOLVER" << std::endl;    
            // Modify the json struct copied from the function parameter
            json_req["STATUS"] = false;
        } else {
            std::cout << "SUCCESSFUL SOLVER RESET" << std::endl;
            json_req["STATUS"] = true;
        }

        // Send reply message
        const std::string reply_msg = json_req.dump();
        zmq::message_t reply{reply_msg.cbegin(), reply_msg.cend()};
        ipc_sock_.send(reply, zmq::send_flags::none);        
    }



    /*** Improves the policy and sends in the next best action to perform ***/
    void sendNextAction(json json_req){
        // Modify local copy of the json struct to send reply message
        json_req["TYPE"] = "NEXT_ACTION_ACK";
        std::cout << "Improving policy" << std::endl;
        // Improve policy without timeout
        if(!solver_->improvePolicy(planningTime_)){
            json_req["STATUS"] = false;
        } else {
            // Policy was improved. Check for next best action
            // Attempt to query next best action
            lastAction_ = solver_->getNextAction();
            // Check if best action is null
            if (!lastAction_) {
                std::cout << "NO BEST ACTION FOUND BY SOLVER" << std::endl;
                json_req["STATUS"] = false;
            } else {
                // Get the vector form of the action and return it as a python list
                VectorFloat actionVec = lastAction_.get()->as<VectorAction>()->asVector();
                printVector(actionVec, "Best action was");
                json_req["LONGITUD_ACT"] = actionVec[0];
                json_req["HORIZONTAL_ACT"] = actionVec[1];
                json_req["STATUS"] = true;
            }
        }

        // Send reply message
        const std::string reply_msg = json_req.dump();
        zmq::message_t reply{reply_msg.cbegin(), reply_msg.cend()};
        ipc_sock_.send(reply, zmq::send_flags::none);   
    }


    /*** Updates believed based on the observation provided in the req message ***/
    void updateCurrentBelief(json json_req){
        // Modify local copy of the json struct to send reply message
        json_req["TYPE"] = "UPDATE_BELIEF_ACK";

        // Parse observation from message
        VectorFloat obsSeen{json_req["REL_LONGIT"], json_req["REL_HORIZONTAL"]};

        bool terminalReached = json_req["TERMINAL"];

        
        // Check if last action is null
        if(!lastAction_){
            std::cout << "LAST ACTION WAS NULL. CANNOT PROCEED" << std::endl;
            json_req["STATUS"] = false;
        } else {
            // Attempt to update belief
            ObservationSharedPtr latestObservation = std::make_shared<NCAPCollisionObservation>(obsSeen);
            // Update the belief
            if (!solver_->updateBelief(lastAction_, latestObservation, terminalReached)) {
                std::cout << "COULDN'T UPDATE BELIEF" << std::endl;
                json_req["STATUS"] = false;
            } else {  
                std::cout << "BELIEF SUCCESSFULLY UPDATED" << std::endl;
                json_req["STATUS"] = true;
            }
        }

        updateViewer();

        // Send reply
        const std::string reply_msg = json_req.dump();
        zmq::message_t reply{reply_msg.cbegin(), reply_msg.cend()};
        ipc_sock_.send(reply, zmq::send_flags::none); 
    }


    /*** Compares if two strings are equal ***/
    bool compareStrings(std::string s1, std::string s2){
        if(s1.compare(s2) == 0){
            return true;
        }

        // Strings are not equal
        return false;
    }




};

}

#endif