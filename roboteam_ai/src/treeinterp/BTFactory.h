//
// Created by baris on 04/10/18.
//

#ifndef ROBOTEAM_AI_BTFACTORY_H
#define ROBOTEAM_AI_BTFACTORY_H

#include "json.h"
#include <iostream>
#include <fstream>
#include <string>
#include <gtest/gtest_prod.h>
#include <map>
#include "TreeInterpreter.h"
#include "BTImport.h"




class BTFactory {

    // TODO: have the names of all the project before here
    TreeInterpreter interpreter;

    public:

        void init();
        static BTFactory& getFactory();
        std::map<std::string,  bt::BehaviorTree> getProject(std::string projectName);
        void updateProject(std::string projectName);
        void updateTree(std::string projectName, std::string treeName);
    private:

    protected:

};


#endif //ROBOTEAM_AI_BTFACTORY_H
