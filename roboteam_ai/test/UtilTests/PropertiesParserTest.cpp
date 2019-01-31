//
// Created by thijs on 2-11-18.
//

#include <utility>
#include <gtest/gtest.h>
#include "roboteam_ai/src/treeinterp/json.h"
#include "roboteam_ai/src/treeinterp/TreeInterpreter.h"

TEST(PropertiesParserTest, ParseCorrectVars) {

    using json = nlohmann::json;
    JsonReader reader;
    json jason = reader.readJSON("propertiesparsertest");
    PropertiesParser parser;
    bt::Blackboard::Ptr bb = parser.parse(jason);

    EXPECT_EQ(bb->getDouble("double_a"), 7.0);
    EXPECT_DOUBLE_EQ(bb->getDouble("double_b"), 3.14159);
    EXPECT_EQ(bb->getInt("int_a"), - 8);
    EXPECT_EQ(bb->getInt("int_b"), 8);
    EXPECT_EQ(bb->getBool("bool_a"), true);
    EXPECT_EQ(bb->getBool("bool_b"), false);
    EXPECT_EQ(bb->getVector2("vector_a").x, 43.0);
    EXPECT_EQ(bb->getVector2("vector_a").y, 41);
    EXPECT_EQ(bb->getVector2("vector_b").x, - 6.8);
    EXPECT_EQ(bb->getVector2("vector_b").y, 3.2);
    EXPECT_EQ(bb->getVector2("vector_c").x, - 1.0);
    EXPECT_EQ(bb->getVector2("vector_c").y, - 2);
    EXPECT_EQ(bb->getString("string_a"), "some string");
    EXPECT_EQ(bb->getString("not_a_bool"), " true ");
    EXPECT_EQ(bb->getString("not_an_int"), " 155 3");
    EXPECT_EQ(bb->getString("not_a_vector"), "{ 4.6 , 6.3.7}");
    EXPECT_EQ(bb->getString("not_a_double"), " 1.3.0");
    EXPECT_EQ(bb->getString("also_not_a_vector"), "{ 5, 8}, 8}");
    EXPECT_EQ(bb->getString("no_vectors_okay?"), "{{ 2, 4 }}");

}
