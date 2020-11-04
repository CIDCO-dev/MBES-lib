/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   StringUtilsTest.hpp
 * Author: emile
 *
 * Created on May 27, 2019, 9:14 AM
 */
#include "../src/utils/StringUtils.hpp"
#include "catch.hpp"

TEST_CASE("Test the String Utils end with function")
{
    const char* text = NULL;
    const char* end = NULL;
    REQUIRE(StringUtils::ends_with(text,end)==false);
    text = "text";
    end = "texte";
    REQUIRE(StringUtils::ends_with(text,end)==false);
    text = "text";
    end = "re";
    REQUIRE(StringUtils::ends_with(text,end)==false);
    text = "text";
    end = "ext";
    REQUIRE(StringUtils::ends_with(text,end));
}

TEST_CASE("Test the String Utils end with ci function")
{
    const char* text = NULL;
    const char* end = NULL;
    REQUIRE(StringUtils::ends_with(text,end)==false);
    text = "tExT";
    end = "texte";
    REQUIRE(StringUtils::ends_with(text,end)==false);
    text = "text";
    end = "re";
    REQUIRE(StringUtils::ends_with(text,end)==false);
    text = "text";
    end = "eXt";
    REQUIRE(StringUtils::ends_with(text,end));
    text = "TeXt";
    end = "ExT";
    REQUIRE(StringUtils::ends_with(text,end));
}

TEST_CASE("Test the String Utils trim function")
{
    const std::string text = "             te xt                 ";
    REQUIRE(StringUtils::trim(text)=="te xt");
}

