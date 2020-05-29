#include <ros_type_introspection/renamer.hpp>

int main( int argc, char** argv)
{
    using namespace RosIntrospection;

    StringTree tree;
    std::vector<SString> vect;

#if !STATIC_TREE
    vect =  {"A", "B", "1", "C", "D"};
    tree.insert( vect );
    vect =  {"A", "B", "1", "E", "F"};
    tree.insert( vect );
    vect =  {"A", "B", "1", "E", "G"};
    tree.insert( vect );
    vect =  {"A", "B", "Y", "ONE"};
    tree.insert( vect );
    vect = {"A", "B", "1", "C", "B", "1"};
    tree.insert( vect );
    vect = {"A", "Y", "B", "1", "C"};
    tree.insert( vect );
    vect = {"B", "1"};
    tree.insert( vect );
#else

#endif


   // bool ret = FindPatternHead(vect, 0, tree.croot(), head);

    std::cout << tree << std::endl;

   /* std::cout << ret << std::endl;
    if(ret)
    {

    }*/



    std::vector<const  StringTreeNode*> heads;

    vect =  { "B", "1"};

    //FindPattern(vect,0, tree.croot() ,heads);
    std::cout << "Found: "<< heads.size() << std::endl;

    for (auto head: heads)
    {
        std::cout << std::make_pair( tree.croot(), head) << std::endl;
    }

    return 0;
}
