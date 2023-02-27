#include <ros/ros.h>
#include <hippo_chain/include/chain_planner/ChainPlanner.h>
#include <boost/program_options.hpp>


namespace po = boost::program_options;

struct PlannerOptions
{
    std::vector<std::string> vehicles;
    double rate = std::numeric_limits<double>::quiet_NaN();
};

PlannerOptions parseArgs(int argc, char **argv)
{
    PlannerOptions options;

    po::options_description descriptions;

    descriptions.add_options()
        ("help,h", "produce help message")
        ("rate,r", po::value<double>(), "rate of estimator in Hz")
        ("vehicles", po::value< std::vector<std::string> >()->multitoken(), "names of vehicles in chain");
     
    po::variables_map varMap;
     
    try 
    {
        po::store(po::command_line_parser(argc, argv).options(descriptions).run(), varMap);
    }
    catch (boost::program_options::invalid_command_line_syntax& e)
    {
        throw ros::Exception(e.what());
    }
    catch (boost::program_options::unknown_option& e)
    {
        throw ros::Exception(e.what());
    }
 
    if (varMap.count("help")) {
        std::cout << descriptions << std::endl;
        exit(0);
    }

    if (varMap.count("rate"))
        options.rate = varMap["rate"].as<double>();

    if (varMap.count("vehicles"))
    {
        std::vector<std::string> topics = varMap["vehicles"].as< std::vector<std::string> >();
        for (std::vector<std::string>::iterator i = topics.begin(); i != topics.end(); i++) {
            if (i->find(":=") != std::string::npos) break;
            options.vehicles.push_back(*i);
        }
    }
    
    return options;
}


int main(int argc, char **argv)
{
    PlannerOptions options = parseArgs(argc, argv);

    ros::init(argc, argv, "chain_planner");

    ChainPlanner planner(options.vehicles, options.rate);

    planner.spin();

    return EXIT_SUCCESS;
}
