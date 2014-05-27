
#include <map>


double sign(double test){
    return test > 0 ? 1 : -1;
}
/**
 *  \desc MCPActuationRadius is a class that finds tabluated values for 
 *  \returns No return set jointVal[] property of class BCDigit
 */
class MCPActuationRadius{

public:
    /** \desc extensionMap is pointer to the map of joint angles to exptension moment arm for the F/E MCP.
     */
    std::map<double, double> *extentionMap;
    /** \desc flexionMap is pointer to the map of joint angles to flextion moment arm for the F/E MCP.
     */
    std::map<double, double> *flextionMap;
    
    double flextionRadius(double jointAngle)
    {
        double static index = (int)jointAngle;
        double static indexDec = jointAngle - (int)jointAngle;

        if( abs(indexDec) > .75)
        {
            indexDec = 1;
        }else if( (abs(indexDec)) < .25 )
        {
            indexDec = 0;
        }else
        {
            indexDec = 0.5;
        }
        index = index + sign(jointAngle)*indexDec;
        
        if( flextionMap->find(index) != flextionMap->end())
        {
            return flextionMap->find(index)->second;
        }else
        {
            return 0;
        }
    }
    double extentionRadius(double jointAngle)
    {
        double static index = (int)jointAngle;
        double static indexDec = jointAngle - (int)jointAngle;
        
        if( abs(indexDec) > .75)
        {
            indexDec = 1;
        }else if( (abs(indexDec)) < .25 )
        {
            indexDec = 0;
        }else
        {
            indexDec = 0.5;
        }
        index = index + sign(jointAngle)*indexDec;
        
        if( extentionMap->find(index) != extentionMap->end())
        {
            return extentionMap->find(index)->second;
        }else
        {
            return 0;
        }
        
    }
    
};



