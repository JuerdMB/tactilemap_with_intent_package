#include "tactilemap_with_intent_package/MapDataConverter.h"

MapDataConverter::MapDataConverter(){
    // Empty constructor
}

std::vector<uint8_t> MapDataConverter::mapToDataArray(grid_map::GridMap &inputMap, const std::string& layer)
{

    int mapWidth = inputMap.getSize()(0);
    int mapHeight = inputMap.getSize()(1);

    if (inputMap.getSize()(0) != 60)
    {
        ROS_ERROR("mapToDataArray: received map of incorrect dimensions %dx%d, returning empty vector", mapWidth, mapHeight);
        return {};
    }

    if (inputMap.getSize()(1) > 40)
        ROS_INFO("mapToDataArray: received map of correct width but incorrect height %d, cutting of last %d rows", mapHeight, mapHeight-40);

    // IF the amount of vertical rows in the map < 40, we cannot fill the whole screen;
    // We can only loop through the

    int lastRow = 40;

    if (inputMap.getSize()(1) < 40)
    {
        ROS_INFO("mapToDataArray: input map insufficient length %d", mapHeight);
        lastRow = mapHeight;
    }

    std::vector<uint8_t> output_data_vector(300);                           // Empty vector of full screen size
    std::fill(output_data_vector.begin(), output_data_vector.end(), 0); // 0-initialize all values

    // Loop through 'braille cells' (collection of 8 map cells per braille cell), first by row, then by column

    for (int braille_cell = 0; braille_cell < 300; braille_cell++)
    {   
        // Determine the start x & y values of this braille cell in the map
        int column_start = (braille_cell * 2) % 60;
        int row_start = ((braille_cell - braille_cell % 30) / 30) * 4;
        
        // Loop through each of the dots of this cell and write them to a bit in the output_data_vector
        for (int x = column_start; x < column_start + 2; x++)
        { // Loop through first column, then second

            for (int y=row_start; y<row_start + 4 && y<lastRow ; y++)
            { // Loop through each of the for vertical rows per column

                grid_map::Index curIndex(x, y);
                uint8_t curPixel = inputMap.at(layer, curIndex);

                if (curPixel > 1)
                {
                    // ROS_INFO("Cell at %2d,%2d > 1, constrained to 1", x, y);
                    curPixel = 1;
                }

                if (curPixel < 0)
                {
                    // ROS_INFO("Cell at %2d,%2d < 0, constrained to 0", x, y);
                    // Got an uncertainty cell
                    curPixel = 0;
                }

                // Determine current bit position in byte, and push the bit into the byte
                uint8_t curBitLoc = (x - column_start) * 4 + (y - row_start); // If in second row, add 4 to y to obtain location
                output_data_vector[braille_cell] |= curPixel << (7 - curBitLoc);
            }
        }
    }
    
    // Printing data for test
    // for(int i =0; i<300; i++){
    //     std::printf("%3d ", output_data_vector[i]);
    //     if( (i+1)%30 == 0 ) std::printf("\n");
    // } std::printf("\n");

    return output_data_vector;
}

void MapDataConverter::mapToScreenResolution(grid_map::GridMap &inputMap, grid_map::GridMap &outputMap)
{
    // Calculate the resolution for total size of 60x40 px
    double res = inputMap.getLength()(0) / 60.;

    // Implementation for resolution change
    grid_map::GridMapCvProcessing::changeResolution(inputMap, outputMap, res);

    ROS_INFO("Resized map from %2dx%2d cells (res=%.3f) to %2dx%2d cells (res=%.3f)",
             inputMap.getSize()(0), inputMap.getSize()(1), inputMap.getResolution(),
             outputMap.getSize()(0), outputMap.getSize()(1), outputMap.getResolution());
}
