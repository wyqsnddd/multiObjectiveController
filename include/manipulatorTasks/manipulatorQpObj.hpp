# ifndef MANIPULATORQPOBJ_HPP
# define MANIPULATORQPOBJ_HPP

# include <vector>
# include <Eigen/Dense>
# include <dart/dart.hpp>


# include <dart/optimizer/Function.hpp>
# include <dart/optimizer/Problem.hpp>

		      
# include <boost/property_tree/ptree.hpp>
# include <boost/property_tree/json_parser.hpp>

//# include <controllers/manipulatorQpController.hpp>

# include <manipulatorTasks/metaManipulatorTask.hpp>
# include <manipulatorTasks/positionTask.hpp>
# include <manipulatorTasks/linearVelocityTask.hpp>
# include <manipulatorTasks/orientationTask.hpp>
# include <utils/utils.hpp>


class manipulatorQpObj: public dart::optimizer::Function{
	public:
		manipulatorQpObj( const dart::dynamics::SkeletonPtr & skelPtr, const pt::ptree configurationDataTree ):dart::optimizer::Function(){
				       
			// qpControllerPtr_ = qpControllerPtr;
			robotPtr_ = skelPtr;
			configurationDataTree_ = configurationDataTree;
			dof_ = robotPtr_->getNumDofs(); 


			initializeTasks_();
			initializeObjMatricies_();
		}

		~manipulatorQpObj(){}
		double eval(const Eigen::VectorXd & _x) const override;
		void evalGradient(const Eigen::VectorXd & _x,
				Eigen::Map<Eigen::VectorXd> _grad ) const override;
		void update();
	private:
		// const manipulatorQpController * qpControllerPtr_;
		dart::dynamics::SkeletonPtr robotPtr_;
		pt::ptree configurationDataTree_;
		/*
		   template <typename T>
		   std::vector<T> as_vector_(pt::ptree const& ptIn, pt::ptree::key_type const& key)
		   {
		   std::vector<T> r;
		   for (auto& item : ptIn.get_child(key))
		   r.push_back(item.second.get_value<T>());
		   return r;
		   }
		   */
		std::vector<std::shared_ptr<metaManipulatorTask> > tasks_;
		void initializeTasks_();

		Eigen::MatrixXd jointWeights_Q_;

		Eigen::MatrixXd objQ_;
		Eigen::VectorXd objP_;
		double objC_;
		int getDof_()const{
			return dof_;	
		}
		int dof_;

		void initializeObjMatricies_(){
	
			objP_ = Eigen::VectorXd::Zero(getDof_());

			objQ_ = Eigen::MatrixXd::Zero(getDof_(), getDof_());
			addToQ_(jointWeights_Q_);

			objC_ = 0.0;
		}
		Eigen::MatrixXd readQ_()const{
			return objQ_;	
		}
		Eigen::VectorXd readP_()const{
			return objP_;	
		}
		double readC_()const{
			return objC_;
		}
		void addToQ_(const Eigen::MatrixXd & inputQ){
			objQ_ = objQ_ + inputQ;	
		}
		void addToP_(const Eigen::VectorXd & inputP){
			objP_ = objP_ + inputP;
		}
		void addToC_(const double inputC){
			objC_ = objC_ + inputC;	
		}

};

# endif
