#ifndef RADIATION_MAPPING_RADIATION_MAPPING_NODE_H
#define RADIATION_MAPPING_RADIATION_MAPPING_NODE_H

class RadiationMapper {
public:
    /**
     * Constructor for the node with the initiation procedure of all systems
     */
    RadiationMapper();

    /**
     * Shutdown procedure for the node
     */
    static void sigintHandler(int sig);
};

#endif //RADIATION_MAPPING_RADIATION_MAPPING_NODE_H