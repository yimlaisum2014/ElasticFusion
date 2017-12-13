/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef ROSBAGREADER_H_
#define ROSBAGREADER_H_

#include <Utils/Resolution.h>
#include <Utils/Stopwatch.h>
#include <pangolin/utils/file_utils.h>

#include "LogReader.h"

#include <cassert>
#include <zlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stack>

class ROSBagReader : public LogReader
{
    public:
        ROSBagReader(std::string file, bool flipColors) : LogReader(file, flipColors){};

        virtual ~ROSBagReader(){};

        void getNext(){};

        void getBack(){};

        int getNumFrames(){return 1;};

        bool hasMore(){return false;};

        bool rewound(){return false;};

        void rewind(){};

        void fastForward(int frame){};

        const std::string getFile(){return "file";};

        void setAuto(bool value){};

        std::stack<int> filePointers;

    private:
        void getCore(){};
};

#endif /* ROSBAGREADER_H_ */
