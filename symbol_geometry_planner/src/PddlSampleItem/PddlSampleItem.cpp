#include "PddlSampleItem.h"
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <pddl_msgs/PDDLPlannerAction.h>

using namespace std::placeholders;
using namespace cnoid;

namespace cnoid {

  void PddlSampleItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<PddlSampleItem>("PddlSampleItem");
  }


  PddlSampleItem::PddlSampleItem()
    : PlannerBaseItem()
  {
  }


  PddlSampleItem::PddlSampleItem(const PddlSampleItem& org)
    : PlannerBaseItem(org)
  {
  }



  PddlSampleItem::~PddlSampleItem()
  {
  }


  void PddlSampleItem::main()
  {
    if(!ros::isInitialized()){
      int argc=0;
      char** argv=0;
      ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
    }

    if(!ros::master::check()){
      this->mv->putln(MessageView::WARNING, "The ROS master is not found.");
      return;
    }

    //nh = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("choreonoid"));

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<pddl_msgs::PDDLPlannerAction> ac("pddl_planner", true);

    this->mv->putln("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    this->mv->putln("Action server started, sending goal.");
    // send a goal to the action
    pddl_msgs::PDDLPlannerGoal goal;

    goal.domain.name = "manip";
    goal.domain.requirements = ":typing";
    goal.domain.types = {"object"};
    goal.domain.predicates = {"(on ?obj0 ?obj1 - object)",
                              "(clear ?obj - object)",
                              "(ontable ?obj - object)",
                              "(holding ?obj - object)",
                              "(handempty)"};
    pddl_msgs::PDDLAction pickup;
    pickup.name = "pickup";
    pickup.parameters = "(?obj - object)";
    pickup.precondition = "(and (ontable ?obj) (clear ?obj) (handempty))";
    pickup.effect = "(and (not (ontable ?obj)) (not (clear ?obj))  (not (handempty)) (holding ?obj))";
    pddl_msgs::PDDLAction putdown;
    putdown.name = "putdown";
    putdown.parameters = "(?obj - object)";
    putdown.precondition = "(and (holding ?obj))";
    putdown.effect = "(and (not (holding ?obj)) (ontable ?obj) (clear ?obj) (handempty))";
    pddl_msgs::PDDLAction stack;
    stack.name = "stack";
    stack.parameters = "(?obj0 ?obj1 - object)";
    stack.precondition = "(and (holding ?obj0) (clear ?obj1))";
    stack.effect = "(and (not (holding ?obj0)) (not (clear ?obj1)) (handempty) (on ?obj0 ?obj1) (clear ?obj0))";
    pddl_msgs::PDDLAction unstack;
    unstack.name = "unstack";
    unstack.parameters = "(?obj0 ?obj1 - object)";
    unstack.precondition = "(and (handempty) (on ?obj0 ?obj1) (clear ?obj0))";
    unstack.effect = "(and (not (handempty)) (not (on ?obj0 ?obj1)) (not (clear ?obj0)) (holding ?obj0) (clear ?obj1))";
    goal.domain.actions = {pickup, unstack, stack, putdown};
    goal.problem.name = "sample";
    goal.problem.domain = "manip";
    pddl_msgs::PDDLObject a; a.name = "a"; a.type="object";
    pddl_msgs::PDDLObject b; b.name = "b"; b.type="object";
    pddl_msgs::PDDLObject c; c.name = "c"; c.type="object";
    goal.problem.objects = {a,b,c};
    goal.problem.initial = {"(on c a)", 
                            "(ontable a)",
                            "(ontable b)",
                            "(clear b)",
                            "(clear c)",
                            "(handempty)"};
    goal.problem.goal = "(and (on a b) (on b c))";

    ros::Time before = ros::Time::now();
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    ros::Time after = ros::Time::now();
    ros::Duration time = after - before;
    this->mv->putln("time: " + std::to_string(time.sec) + "." + std::to_string(time.nsec));

    if (finished_before_timeout)
      {

        actionlib::SimpleClientGoalState state = ac.getState();
        this->mv->putln("Action finished: " + state.toString());
        pddl_msgs::PDDLPlannerResultConstPtr result = ac.getResult();
        std::stringstream str;
        str << *result;
        this->mv->putln("Result: " + str.str());
      }
    else
      this->mv->putln(MessageView::WARNING, "Action did not finish before the time out.");

  }
}
