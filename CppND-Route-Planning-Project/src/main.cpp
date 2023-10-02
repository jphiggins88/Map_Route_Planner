#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
//#include "../IO2D/P0267_RefImpl/P0267_RefImpl/P0267_RefImpl/cairo/sdl2/io2d.h"
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

/**
 * @brief Read a file and return the contents
 *
 * Accepts a reference to a path and attempts to read using an ifstream.
 * If the file is empty or does not exist, nullopt is returned.
 *
 * @param path Reference to the path of the file to be read
 * @return (optional) vector<byte> containing the contents of the file, or nullopt if file is empty or does not exist.
 */
static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

//TODO Add Doxygen comments and add parsing to the input to make sure appropriate values are entered. (range checking and positive ints/floats only.)
static void getUserCoordinates(float& start_x, float& start_y, float& end_x, float& end_y)
{
    std::cout << "Enter starting coordinates (separated by a space):\n";
    std::cin >> start_x >> start_y;
    std::cout << "Enter ending coordinates (separated by a space):\n";
    std::cin >> end_x >> end_y;
}


int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";

    // Allow user to start the executable with an argument to load a custom map
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            // string_view provides a non-owning view into a sequence of characters
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = "../maps/" + std::string{argv[i]};
    }
    // Load the default map
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;

        // Path to the default map
        osm_data_file = "../maps/map_Irvine1.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    // Read the map file
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
                osm_data = std::move(*data);
    }
    
    //TODO, give the option eventually to point and click 2 spots on the map.
    // Get user input in individual coordinates
    float start_x, start_y, end_x, end_y;
    getUserCoordinates(start_x, start_y, end_x, end_y);

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    // Use IO2D to render the map
    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
