#include <ros/ros.h>
#include <hippo_chain/include/chain_watcher/ChainWatcher.h>
#include <boost/program_options.hpp>


namespace po = boost::program_options;

struct WatcherOptions
{
    std::vector<std::string> vehicles;
};

WatcherOptions parseArgs(int argc, char **argv)
{
    WatcherOptions options;

    po::options_description descriptions;

    descriptions.add_options()
        ("help,h", "produce help message")
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
    WatcherOptions options = parseArgs(argc, argv);

    ros::init(argc, argv, "chain_watcher");

    ChainWatcher watcher(options.vehicles);

    watcher.spin();

    return EXIT_SUCCESS;
}
