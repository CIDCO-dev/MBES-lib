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

TEST_CASE("Test the String Utils ends_with_ci function") {
    std::string file_s7k = "file.s7k";
    std::string file_xtf = "file.xtf";
    std::string file_all = "file.all";
    
    std::string file_s7k_cap = "file.S7K";
    std::string file_xtf_cap = "file.XTF";
    std::string file_all_cap = "file.ALL";
    
    std::string file_s7k_mixedCase = "file.s7K";
    std::string file_xtf_mixedCase = "file.XtF";
    std::string file_all_mixedCase = "file.alL";
    
    std::string ext_s7k = ".s7k";
    std::string ext_xtf = ".xtf";
    std::string ext_all = ".all";
    
    REQUIRE(StringUtils::ends_with_ci(file_s7k.c_str(),ext_s7k.c_str())==true);
    REQUIRE(StringUtils::ends_with_ci(file_s7k.c_str(),ext_xtf.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_s7k.c_str(),ext_all.c_str())==false);
    
    REQUIRE(StringUtils::ends_with_ci(file_xtf.c_str(),ext_s7k.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_xtf.c_str(),ext_xtf.c_str())==true);
    REQUIRE(StringUtils::ends_with_ci(file_xtf.c_str(),ext_all.c_str())==false);
    
    REQUIRE(StringUtils::ends_with_ci(file_all.c_str(),ext_s7k.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_all.c_str(),ext_xtf.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_all.c_str(),ext_all.c_str())==true);
    
    REQUIRE(StringUtils::ends_with_ci(file_s7k_cap.c_str(),ext_s7k.c_str())==true);
    REQUIRE(StringUtils::ends_with_ci(file_s7k_cap.c_str(),ext_xtf.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_s7k_cap.c_str(),ext_all.c_str())==false);
    
    REQUIRE(StringUtils::ends_with_ci(file_xtf_cap.c_str(),ext_s7k.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_xtf_cap.c_str(),ext_xtf.c_str())==true);
    REQUIRE(StringUtils::ends_with_ci(file_xtf_cap.c_str(),ext_all.c_str())==false);
    
    REQUIRE(StringUtils::ends_with_ci(file_all_cap.c_str(),ext_s7k.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_all_cap.c_str(),ext_xtf.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_all_cap.c_str(),ext_all.c_str())==true);
    
    REQUIRE(StringUtils::ends_with_ci(file_s7k_mixedCase.c_str(),ext_s7k.c_str())==true);
    REQUIRE(StringUtils::ends_with_ci(file_s7k_mixedCase.c_str(),ext_xtf.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_s7k_mixedCase.c_str(),ext_all.c_str())==false);
    
    REQUIRE(StringUtils::ends_with_ci(file_xtf_mixedCase.c_str(),ext_s7k.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_xtf_mixedCase.c_str(),ext_xtf.c_str())==true);
    REQUIRE(StringUtils::ends_with_ci(file_xtf_mixedCase.c_str(),ext_all.c_str())==false);
    
    REQUIRE(StringUtils::ends_with_ci(file_all_mixedCase.c_str(),ext_s7k.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_all_mixedCase.c_str(),ext_xtf.c_str())==false);
    REQUIRE(StringUtils::ends_with_ci(file_all_mixedCase.c_str(),ext_all.c_str())==true);
}

TEST_CASE("Test the String Utils strcmpi function") {
    std::string upper = "ABCDEF";
    std::string lower = "abcdef";
    std::string lower_upper = "abcDEF";
    std::string different = "uvwxyz";
    
    REQUIRE(StringUtils::strcmpi(upper, lower) == true);
    REQUIRE(StringUtils::strcmpi(upper, lower_upper) == true);
    REQUIRE(StringUtils::strcmpi(lower, lower_upper) == true);
    
    REQUIRE(StringUtils::strcmpi(upper, different) == false);
    REQUIRE(StringUtils::strcmpi(lower, different) == false);
    REQUIRE(StringUtils::strcmpi(lower_upper, different) == false);
}

TEST_CASE("Test the String Utils ends_with function")
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
    REQUIRE(StringUtils::ends_with(text,end)==false); // FYI ends_with() compares string lengths
    text = "TeXt";
    end = "ExT";
    REQUIRE(StringUtils::ends_with(text,end)==false); // FYI ends_with() compares string lengths
}

TEST_CASE("Test the String Utils trim function")
{
    const std::string text = "             te xt                 ";
    REQUIRE(StringUtils::trim(text)=="te xt");
}

