#include <cstdlib> // std::getenv

#include <iostream> // std::cerr

#include <boost/predef.h> // BOOST_OS_IOS

#if (BOOST_OS_IOS)
# include <regex> // std::regex
# include <unistd.h> // chdir
#endif

// Should be defined by user
int drishti_main(int argc, char** argv);

int main(int argc, char** argv) {
  try {
#if (BOOST_OS_IOS)
    const char* home = std::getenv("HOME");
    if (home == nullptr) {
      std::cerr << "HOME not found" << std::endl;
      return EXIT_FAILURE;
    }

    std::string working_dir(home);
    working_dir += "/tmp";

    if (chdir(working_dir.c_str()) != 0) {
      std::cerr << "Can't change working directory" << std::endl;
      return EXIT_FAILURE;
    }

    std::vector<char*> new_argv(argc);
    const std::regex r("^\\$<DRISHTI_RESOURCE_FILE:(.*)>$");

    std::string fmt(home);
    fmt += "/\\1";

    for (int i = 0; i < argc; ++i) {
      const std::string original(argv[i]);
      const std::string x = std::regex_replace(original, r, fmt, std::regex_constants::format_sed);
      if (original == x) {
        new_argv[i] = argv[i];
      }
      else {
        new_argv[i] = new char[x.size() + 1]; // will leak
        x.copy(new_argv[i], x.size());
        new_argv[i][x.size()] = '\0';
      }
    }

    return drishti_main(argc, new_argv.data());
#else
    // If platform is not iOS - leave as is
    return drishti_main(argc, argv);
#endif
  }
  catch (std::exception& exc) {
    std::cerr << "Exception catched: " << exc.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (...) {
    std::cerr << "Unknown exception catched" << std::endl;
    return EXIT_FAILURE;
  }
}
