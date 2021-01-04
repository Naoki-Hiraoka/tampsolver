#include <multicontact_controller/lib/CnoidBodyUtils/Drawing.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    void drawCoordsLineCoords(cnoid::SgLineSetPtr& lines, const cnoid::Position& A_pos, const cnoid::Position& B_pos){
      if(!lines){
        lines = new cnoid::SgLineSet;
        lines->setLineWidth(1.0);
        lines->getOrCreateColors()->resize(4);
        lines->getOrCreateColors()->at(0) = cnoid::Vector3f(1.0,1.0,1.0);
        lines->getOrCreateColors()->at(1) = cnoid::Vector3f(1.0,0.0,0.0);
        lines->getOrCreateColors()->at(2) = cnoid::Vector3f(0.0,1.0,0.0);
        lines->getOrCreateColors()->at(3) = cnoid::Vector3f(0.0,0.0,1.0);
        // A, A_x, A_y, A_z, B, B_x, B_y, B_z
        lines->getOrCreateVertices()->resize(8);
        lines->colorIndices().resize(0);
        lines->addLine(0,1); lines->colorIndices().push_back(1); lines->colorIndices().push_back(1);
        lines->addLine(0,2); lines->colorIndices().push_back(2); lines->colorIndices().push_back(2);
        lines->addLine(0,3); lines->colorIndices().push_back(3); lines->colorIndices().push_back(3);
        lines->addLine(4,5); lines->colorIndices().push_back(1); lines->colorIndices().push_back(1);
        lines->addLine(4,6); lines->colorIndices().push_back(2); lines->colorIndices().push_back(2);
        lines->addLine(4,7); lines->colorIndices().push_back(3); lines->colorIndices().push_back(3);
        lines->addLine(0,4); lines->colorIndices().push_back(0); lines->colorIndices().push_back(0);

      }

      lines->getOrCreateVertices()->at(0) = A_pos.translation().cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(1) = (A_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(2) = (A_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(3) = (A_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(4) = B_pos.translation().cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(5) = (B_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(6) = (B_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(7) = (B_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();

    }

    void drawPolygon(cnoid::SgLineSetPtr& lines, const std::vector<cnoid::Vector2>& vertices, const cnoid::Vector3f color){
      if(!lines){
        lines = new cnoid::SgLineSet;
        lines->setLineWidth(3.0);
        lines->getOrCreateColors()->resize(1);
        lines->getOrCreateColors()->at(0) = color;

        lines->getOrCreateVertices()->resize(0);
        lines->colorIndices().resize(0);
      }

      lines->getOrCreateVertices()->resize(vertices.size());
      lines->colorIndices().resize(vertices.size());
      lines->setNumLines(vertices.size());
      for(size_t i=0;i<vertices.size();i++){
        int next = (i+1 != vertices.size())? i+1 : 0;
        lines->vertices()->at(i) = cnoid::Vector3f(vertices[i][0],vertices[i][1],0);
        lines->colorIndices()[i] = 0;
        lines->line(i)[0] = i;
        lines->line(i)[1] = next;
      }
    }
  };
};
