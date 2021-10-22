#include "CFRCalculator.h"

#include <iostream>
#include <static_equilibuim_test/StaticEquilibuimTest.h>
#include <cnoid/EigenUtil>

namespace CFR {
  bool appendRow(const std::vector<cnoid::VectorX>& vs, cnoid::VectorX& vout){
    size_t rows = 0;
    for(size_t i=0;i<vs.size();i++){
      rows += vs[i].size();
    }
    vout.resize(rows);
    size_t idx = 0;
    for(size_t i=0;i<vs.size();i++){
      vout.segment(idx,vs[i].size()) = vs[i];
      idx += vs[i].size();
    }

    return true;
  }

  bool appendCol(const std::vector<Eigen::SparseMatrix<double, Eigen::ColMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::ColMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(Mout.rows(),0);//もとのMoutのrowを用いる
      return true;
    }
    size_t rows = Ms[0].rows();
    size_t cols = 0;
    for(size_t i=0;i<Ms.size();i++){
      cols += Ms[i].cols();
      if(Ms[i].rows() != rows){
        std::cerr << "[appendCol] Ms[i].rows() " << Ms[i].rows() << " != rows " << rows << std::endl;
        return false;
      }
    }
    Mout.resize(rows,cols);
    size_t idx = 0;
    for(size_t i=0;i<Ms.size();i++){
      Mout.middleCols(idx,Ms[i].cols()) = Ms[i];
      idx += Ms[i].cols();
    }

    return true;
  }

  bool appendDiag(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(0,0);
      return true;
    }
    size_t cols = 0;
    size_t rows = 0;
    for(size_t i=0;i<Ms.size();i++){
      rows += Ms[i].rows();
      cols += Ms[i].cols();
    }
    Mout.resize(rows,cols);
    size_t idx_row = 0;
    size_t idx_col = 0;
    for(size_t i=0;i<Ms.size();i++){
      Eigen::SparseMatrix<double, Eigen::ColMajor> M_ColMajor(Ms[i].rows(),cols);
      M_ColMajor.middleCols(idx_col,Ms[i].cols()) = Ms[i];
      Mout.middleRows(idx_row,M_ColMajor.rows()) = M_ColMajor;
      idx_row += Ms[i].rows();
      idx_col += Ms[i].cols();
    }

    return true;
  }

  bool CFRCalculator::computeCFR(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, cnoid::BodyPtr& robot, int debugLevel) {
    std::vector<std::shared_ptr<primitive_motion_level_tools::PrimitiveState> > supportEEFs;
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++) {
      if(it->first != "com" && it->second->supportCOM()){
        supportEEFs.push_back(it->second);
      }
    }

    if(supportEEFs.size() ==0) return false;

    Eigen::SparseMatrix<double,Eigen::RowMajor> A;
    Eigen::VectorXd b;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C;
    Eigen::VectorXd dl;
    Eigen::VectorXd du;
    {
      // compute A, b, C, dl, du
      // x = [dpx dpy dw1 dw2 ...]^T
      // wはエンドエフェクタ座標系．エンドエフェクタまわり. (isWrenchCGlobal=false)サイズは6
      // wはworld座標系．world原点まわり. (isWrenchCGlobal=true)サイズは6

      // Grasp Matrix Gx = h
      // 原点まわりのつりあい. 座標軸はworld系の向き
      Eigen::SparseMatrix<double,Eigen::RowMajor> G;
      Eigen::VectorXd h = Eigen::VectorXd::Zero(6);
      {
        h[2] = robot->mass()*9.80665;

        std::vector<Eigen::SparseMatrix<double,Eigen::ColMajor> > Gs;
        {
          Eigen::SparseMatrix<double,Eigen::ColMajor> G01(6,2);
          G01.insert(3,1) = -robot->mass()*9.80665;
          G01.insert(4,0) = robot->mass()*9.80665;
          Gs.push_back(G01);
        }

        for(size_t i=0;i<supportEEFs.size();i++){
          Eigen::SparseMatrix<double,Eigen::ColMajor> GraspMatrix(6,6);
          {
            const cnoid::Position& pos = (supportEEFs[i]->isWrenchCGlobal()) ? cnoid::Position::Identity() : supportEEFs[i]->targetPose();
            const cnoid::Matrix3& R = pos.linear();
            const cnoid::Matrix3& p_x_R = cnoid::hat(pos.translation()) * R;

            std::vector<Eigen::Triplet<double> > G_tripletList;
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(row,col,R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(3+row,col,p_x_R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                  G_tripletList.push_back(Eigen::Triplet<double>(3+row,3+col,R(row,col)));
              }
            }
            GraspMatrix.setFromTriplets(G_tripletList.begin(), G_tripletList.end());
          }
          Gs.push_back(GraspMatrix);
        }

        Eigen::SparseMatrix<double,Eigen::ColMajor> G_ColMajor;
        CFR::appendCol(Gs,G_ColMajor);
        G = G_ColMajor;
      }

      //接触力制約
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_contact;
      Eigen::VectorXd b_contact;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_contact;
      Eigen::VectorXd dl_contact;
      Eigen::VectorXd du_contact;
      {
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
        std::vector<Eigen::VectorXd> bs;
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
        std::vector<Eigen::VectorXd> dls;
        std::vector<Eigen::VectorXd> dus;

        {
          Eigen::SparseMatrix<double,Eigen::RowMajor> A01(0,2);
          Eigen::VectorXd b01(0);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C01(0,2);
          Eigen::VectorXd dl01(0);
          Eigen::VectorXd du01(0);
          As.push_back(A01);
          bs.push_back(b01);
          Cs.push_back(C01);
          dls.push_back(dl01);
          dus.push_back(du01);
        }

        for (size_t i=0;i<supportEEFs.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A(0,6);
          Eigen::VectorXd b(0);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C = supportEEFs[i]->wrenchC();
          Eigen::VectorXd dl = supportEEFs[i]->wrenchld();
          Eigen::VectorXd du = supportEEFs[i]->wrenchud();
          As.push_back(A);
          bs.push_back(b);
          Cs.push_back(C);
          dls.push_back(dl);
          dus.push_back(du);
        }
        CFR::appendDiag(As,A_contact);
        CFR::appendRow(bs,b_contact);
        CFR::appendDiag(Cs,C_contact);
        CFR::appendRow(dls,dl_contact);
        CFR::appendRow(dus,du_contact);
      }

      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(G.rows()+A_contact.rows(),G.cols());
      A.topRows(G.rows()) = G;
      A.bottomRows(A_contact.rows()) = A_contact;
      b = Eigen::VectorXd(h.size()+b_contact.size());
      b.head(h.rows()) = h;
      b.tail(b_contact.rows()) = b_contact;
      C = C_contact;
      dl = dl_contact;
      du = du_contact;
    }

    // SCFRの各要素はx[0:1]の次元の大きさに揃っている
    if(!static_equilibuim_test::calcProjection(A,b,C,dl,du,this->M_,this->l_,this->u_,this->vertices_,debugLevel, 0.001,30,false)){
      std::cerr << "[CFRCalculator::computeCFR] projection failed" << std::endl;
      this->M_.resize(0,2);
      this->l_.resize(0);
      this->u_.resize(0);
      this->vertices_.clear();
      return false;
    }

    return true;
  }
}
