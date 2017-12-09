#include "8_innovation_script.h"
CInnovationScript::CInnovationScript()
{
    
}

CInnovationScript::~CInnovationScript()
{

}

void CInnovationScript::Queue()
{
    stAct newAct;

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "创新创意 项目开始";
    newAct.nDuration = 3;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "机器人";
    arAct.push_back(newAct);
    
    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "你好";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

}