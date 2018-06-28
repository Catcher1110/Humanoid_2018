#ifndef VALKYRIE_STATE_PROVIDER
#define VALKYRIE_STATE_PROVIDER

#include <Utils/wrap_eigen.hpp>
#include <Valkyrie/Valkyrie_Definition.h>

class Valkyrie_StateProvider{
    public:
        dynacore::Vector q_;
        dynacore::Vector qdot_;

        static Valkyrie_StateProvider* getStateProvider(){
            static Valkyrie_StateProvider sp;
            return & sp;
        }

        int stance_foot_;
        dynacore::Vect3 global_stance_foot_pos_;
        dynacore::Vector Fr_;
    private:
        Valkyrie_StateProvider():q_(valkyrie::num_q), qdot_(valkyrie::num_qdot){}
};
#endif
