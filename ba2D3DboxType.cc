
#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres_functions.h"

using namespace std;

double* object_half_size;
double* object_weight;
double fx, fy, px, py, w3Dv2D;
double N[4]; //={0,1,0,100};
int exe_time=0;

int main(int argc, char** argv)
{
	std::cout<<"Ba2D3D bundle adjuster in 2D and 3D. Writen by Jianxiong Xiao."<<std::endl;
	std::cout<<"Usage: EXE mode(1,2,3,5) w3Dv2D input_file_name output_file_name"<<std::endl;


	unsigned int n_cameras, n_points, n_observations, n_objects, n_rand_pairs, n_selected = 0;
	unsigned int *object_type, *random_array;

	unsigned int *point_observed_index;
	double *point_observed_value, *camera_matrix, *object_rot_trans, *point_cloud;
	double *camera_parameter;
	double *object_parameter;
	double *box_constraints_var;

	double init_box_bnd =	5.0;
	int mode            =	atoi(argv[1]);
	w3Dv2D              =	atof(argv[2]);
	bool verbose_flag   =	true;

	// start reading input file
	FILE* fp = fopen(argv[3],"rb");
	if ( fp == NULL ) 
	{ 
		cout<<"fail to open file"<<std::endl; return false;
	}

	// read header count
	fread( (void*) (&n_cameras), sizeof( unsigned int ), 1, fp );
	fread( (void*) (&n_points), sizeof( unsigned int ), 1, fp );
	fread( (void*) (&n_observations), sizeof( unsigned int ), 1, fp );
	fread( (void*) (&n_objects), sizeof( unsigned int ), 1, fp );
	fread( (void*) (&n_rand_pairs), sizeof( unsigned int ), 1, fp );

	// read camera intrinsic
	fread( (void*)(&fx), sizeof( double ), 1, fp);
	fread( (void*)(&fy), sizeof( double ), 1, fp);
	fread( (void*)(&px), sizeof( double ), 1, fp);
	fread( (void*)(&py), sizeof( double ), 1, fp);
	
	fread( (void*)(&N), sizeof( double ), 4,fp );

	/* Memory Allocation being done here */
	point_observed_index    =	new unsigned int [ 2*n_observations ];
	point_observed_value    =	new double [ 6*n_observations ];
	camera_matrix           =	new double [ 12*n_cameras ];
	object_rot_trans        =	new double [ 12*n_objects ];
	point_cloud             =	new double [ 3*n_points ];
	camera_parameter        =	new double [ 6*n_cameras ];
	object_parameter        =	new double [ 6*n_objects ];
	box_constraints_var     =	new double [ 3*n_rand_pairs*n_rand_pairs ]();
	random_array            =	new	unsigned int [ 2*n_rand_pairs ];

	/* Initialize box_constraints_var */
	for( int idx = 0; idx < ( n_rand_pairs * n_rand_pairs ); idx++ )
		box_constraints_var[ 3*idx ]    =	init_box_bnd, 
		box_constraints_var[ 3*idx+1 ]  =	init_box_bnd,
		box_constraints_var[ 3*idx+2 ]  =	init_box_bnd;

	fread( (void*) (camera_matrix), sizeof(double), 12*n_cameras, fp );                                     //	read camera extrinsics
	fread( (void*) (object_rot_trans), sizeof(double), 12*n_objects, fp );                                  //	read object rotations / translations
	fread( (void*) (point_cloud), sizeof(double), 3*n_points, fp );                                         //	read initial 3D point positions.

	object_half_size        =	new double [ 3*n_objects ];
	object_weight           =	new double [ n_objects ];
	object_type             =	new unsigned int [ n_objects ];

	fread( (void*) (point_observed_index), sizeof(unsigned int), 2*n_observations, fp);                     //	read which point is observed in which image.
	fread( (void*) (point_observed_value), sizeof(double), 6*n_observations, fp);                           //	read observed 3D and 2D points in each image.
	fread( (void*) (object_half_size), sizeof(double), 3*n_objects, fp);                                    //	UNKNOWN, TODO
	fread( (void*) (object_weight), sizeof(double), n_objects, fp);                                         //	UNKNOWN, TODO
	fread( (void*) (object_type), sizeof(unsigned int), n_objects, fp);                                     //	UNKNOWN, TODO
	fread( (void*) (random_array), sizeof(unsigned int), 2*n_rand_pairs, fp);                                     //	UNKNOWN, TODO

	// finish reading
	fclose(fp);

	/* Output read parameters if verbose_flag is turned on */
	if( verbose_flag )
	{
		cout << "Parameters: ";
		cout << "mode = " << mode << " ";
		cout << "w3Dv2D = " << w3Dv2D << "\t"; //<<std::endl;  
		cout << "Meta Info: ";
		cout << "nCam = " << n_cameras << " ";
		cout << "nPts = " << n_points << " ";
		cout << "nObs = " << n_observations << " ";
		cout << "n_objects = " << n_objects << "\t"; //<<std::endl;
		cout << "Camera Intrinsic: ";
		cout << "fx = " << fx << " ";
		cout << "fy = " << fy << " ";
		cout << "px = " << px << " ";
		cout << "N = " << N[0] << " "<< N[1] << " "<< N[2] << " "<< N[3] << " ";		
		cout << "py = " << py << "\t" << endl;
		

        fp = fopen( "edge_pairs.txt", "a");
        fprintf( fp, "New version of edge pairs" );
        for( int tmpidx = 0; tmpidx < n_rand_pairs; tmpidx++ )
            fprintf( fp, "%d %d\n", random_array[tmpidx], random_array[tmpidx+n_rand_pairs] );
        fclose(fp);

		for( int object_id = 0; object_id < n_objects; ++object_id )
		{
            cout << "Object #" << object_id << " type=" << object_type[ object_id ] << " ";
            cout << "weight=" << object_weight[ object_id ] << " ";
            cout << "halfSize=" << object_half_size[ object_id * 3 ] << " " ;
            cout << object_half_size[ object_id * 3 + 1 ] << " " ;
            cout << object_half_size[ object_id * 3 + 2 ] << endl;
		}
	}

	/* Construct ceres cameras and objects from camera matrix and object transformations */
	ceres_fill_camera_parameters( camera_matrix, camera_parameter, n_cameras );
	ceres_fill_object_parameters( object_rot_trans, object_parameter, object_type, n_objects );

	/* 
	 * Create residuals for each objservation in the bundle adjustment problem. The
	 * parameters for cameras and points are added automatically.
	 */
	ceres::Problem problem;
	ceres::LossFunction* loss_function          =	NULL;                                                   // squared loss
	ceres::LossFunction* loss_functionObject    =	NULL;                                                   // new ceres::HuberLoss(1.0);

	/*
	 * First generate a random set of 100 points, and then regenerate these points if necessary
	 */
	generate_random_number_array( random_array, n_rand_pairs );

	for( unsigned int id_obs = 0; id_obs < n_observations; id_obs++ )
	{
		int not_first_frame         =	0;
		double *camera_ptr          =	camera_parameter + \
										point_observed_index[ 2*id_obs ] * 6;
		double *obs_ptr             =	point_observed_value + 6*id_obs;
		double *cam_ptr_one;
		double *cam_ptr_two;

		/* Initialize pointers to three types of cost functions */

		if( point_observed_index[ 2*id_obs ] >= 2 )
		{
			cam_ptr_one             =	camera_parameter + \
										( point_observed_index[ 2*id_obs ]-1 ) * 6;
			cam_ptr_two             =	camera_parameter + \
										( point_observed_index[ 2*id_obs ]-2 ) * 6;
			not_first_frame         =	1;
		}

		//random number generator
		if( obs_ptr[5] < 0 )                                                                                // This is trivially true for matlab box_synthetic.m example
		{
			double* point_ptr_one   =	point_cloud + point_observed_index[ 2*id_obs+1 ] * 3;
            if( !id_obs ) cout << "<-------------------------------- In Mode " << mode << " ----------------------------------->" << endl;

			switch (mode)
			{
				case 1:                                                                                     // 2D triangulation
					ceres_add_triangulation_function( obs_ptr, camera_ptr, point_ptr_one, \
													loss_function, problem );
					break;
				case 2:                                                                                     // 2D bundle adjustment
					ceres_add_alignment_error_2d_function( obs_ptr, camera_ptr, \
													point_ptr_one, loss_function, problem );
					break;
				case 3:                                                                                     // 3D bundle adjustment
					ceres_add_alignment_error_3d_function( obs_ptr, camera_ptr, \
													point_ptr_one, loss_function, problem );
					break;
				case 4:                                                                                     // 3D+2D bundle adjustment
					ceres_add_alignment_error_2d3d_function( obs_ptr, camera_ptr, \
													point_ptr_one, loss_function, problem );
					break;
				case 5:                                                                                     // 3D+2D+Trajectory+normal bundle adjustment
					ceres_add_alignment_traj_normal_error_function( obs_ptr, camera_ptr, \
													point_ptr_one, cam_ptr_one, cam_ptr_two, \
													loss_function, problem, not_first_frame);
					break;
				case 6:                                                                                     //3D+2D+Trajectory
					ceres_add_alignment_traj_error_function( obs_ptr, camera_ptr, \
													point_ptr_one, cam_ptr_one, cam_ptr_two, \
													loss_function, problem, not_first_frame );
					break;
				case 7:                                                                                     // 3D+2D+Trajectory+normal bundle adjustment
					ceres_add_alignment_traj1_normal_error_function( obs_ptr, camera_ptr, \
													point_ptr_one, cam_ptr_one, cam_ptr_two, \
														loss_function, problem, not_first_frame);
					break;
				case 8:                                                                                     //3D+2D+Trajectory
					ceres_add_alignment_traj1_error_function( obs_ptr, camera_ptr, \
													point_ptr_one, cam_ptr_one, cam_ptr_two, \
													loss_function, problem, not_first_frame );
					break;
						
				
				case 9:                                                                                     //3D+2D+Trajectory+box constraints
				case 10:
				case 11:
				case 12:
					n_selected += ceres_add_alignment_traj_normal_box_error_function( obs_ptr, camera_ptr, \
													point_ptr_one, point_cloud, cam_ptr_one, \
													cam_ptr_two, point_observed_index, \
													loss_function, problem, random_array, \
													box_constraints_var, n_points, n_cameras, \
													n_rand_pairs, id_obs, mode, not_first_frame & (n_selected < n_rand_pairs) );
					break;
				case 13:                                                                                     // 3D+2D+normal bundle adjustment
				ceres_add_alignment_error_2d3d_normal_function( obs_ptr, camera_ptr,cam_ptr_one, \
														point_ptr_one, loss_function, problem );
				break;
				case 14:                                                                                     // 2D+normal+traj bundle adjustment
				ceres_add_alignment_2d_traj_normal_error_function( obs_ptr, camera_ptr,cam_ptr_one,\
													cam_ptr_two, point_ptr_one, loss_function, problem, not_first_frame & (n_selected < n_rand_pairs) );
				break;
				case 15:                                                                                     // 3D+normal+traj bundle adjustment
				ceres_add_alignment_3d_traj_normal_error_function( obs_ptr, camera_ptr,cam_ptr_one,cam_ptr_two, \
													point_ptr_one, loss_function, problem , not_first_frame & (n_selected < n_rand_pairs));
				break;
				case 16:                                                                                     //  2D+3D+normal+traj bundle adjustment
				ceres_add_alignment_2d3d_traj_normal_error_function( obs_ptr, camera_ptr,cam_ptr_one,cam_ptr_two, \
													point_ptr_one, loss_function, problem ,not_first_frame);
				break;
				case 17:                                                                                     // 2D+normal bundle adjustment
					ceres_add_alignment_error_2d_normal_function( obs_ptr, camera_ptr,cam_ptr_one,point_ptr_one, loss_function, problem );
					break;
				case 18:                                                                                     //3D+normal bundle adjustment
					ceres_add_alignment_error_3d_normal_function( obs_ptr, camera_ptr,cam_ptr_one,point_ptr_one, loss_function, problem );

			}
		}
		else
		{
			double* object_ptr = object_parameter + int(obs_ptr[5]) * 6;
			ceres::CostFunction* cost_function;

			if( object_weight[ int( obs_ptr[5] ) ] > 0 )
			{
				switch( object_type[ int( obs_ptr[5] ) ] )
				{
					case 0:                                                                                 // a floor
						cost_function = new ceres::AutoDiffCostFunction< AlignmentErrorFloor, 3, 6 > \
                                        ( new AlignmentErrorFloor( obs_ptr ) );
						problem.AddResidualBlock( cost_function, loss_functionObject, camera_ptr );
						break;
					case 1:                                                                                 // an axis y align object, eg. a ceiling, or a wall
						cost_function = new ceres::AutoDiffCostFunction< AlignmentErrorAxisBox, 3, 6, 4 > \
                                        ( new AlignmentErrorAxisBox( obs_ptr ) );
						problem.AddResidualBlock( cost_function, loss_functionObject, camera_ptr, object_ptr+2 );
						break;
					case 2:                                                                                 // a general object
						cost_function = new ceres::AutoDiffCostFunction< AlignmentErrorBox, 3, 6, 6 > \
                                        ( new AlignmentErrorBox( obs_ptr ) );
						problem.AddResidualBlock( cost_function, loss_functionObject, camera_ptr, object_ptr );
						break;
                    case 3:                                                                                 // mirror
						break;
					case 4:                                                                                 // a AlignmentErrorAxisManhattanBox align object
						cost_function = new ceres::AutoDiffCostFunction< AlignmentErrorAxisManhattanBox, 3, 6, 3 > \
                                        ( new AlignmentErrorAxisManhattanBox( obs_ptr ) );
						problem.AddResidualBlock( cost_function, loss_functionObject, camera_ptr, object_ptr+3 );
						break;
				}
			}
		}
	}

	//----------------------------------------------------------------

	// Make Ceres automatically detect the bundle structure. Note that the
	// standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
	// for standard bundle adjustment problems.
	ceres::Solver::Options options;
	options.max_num_iterations					=	1000;  
	options.minimizer_progress_to_stdout		=	true;
	options.linear_solver_type					=	ceres::SPARSE_NORMAL_CHOLESKY;                          //ceres::SPARSE_SCHUR;  //ceres::DENSE_SCHUR;
	options.num_linear_solver_threads = 4;

	/*
	 * Unused options, commented for now.
		options.ordering_type                   =	ceres::SCHUR;
		options.linear_solver_type              =	ceres::DENSE_SCHUR;                                     //ceres::SPARSE_SCHUR; //ceres::DENSE_SCHUR; //ceres::SPARSE_NORMAL_CHOLESKY; //
		options.ordering_type                   =	ceres::SCHUR;
		options.minimizer_progress_to_stdout    =	true;
		options.preconditioner_type             =	ceres::JACOBI;                                          // ceres::IDENTITY
		options.trust_region_strategy_type      =	ceres::LEVENBERG_MARQUARDT;
		options.use_block_amd                   =	true;
		options.eta                             =	1e-2;
		options.dogleg_type                     =	ceres::TRADITIONAL_DOGLEG;
		options.use_nonmonotonic_steps          =	false;

		options.trust_region_strategy_type      =	ceres::LEVENBERG_MARQUARDT;                             // DEFINE_string(trust_region_strategy, "lm", "Options are: lm, dogleg");
		options.eta                             =	1e-2;                                                   // DEFINE_double(eta, 1e-2, "Default value for eta. Eta determines 
                                                                                                            // the accuracy of each linear solve of the truncated newton step. 
                                                                                                            // Changing this parameter can affect solve performance ");
		options.linear_solver_type              =	ceres::SPARSE_SCHUR;                                    // DEFINE_string(solver_type, "sparse_schur", "Options are:  
                                                                                                            // sparse_schur, dense_schur, iterative_schur, sparse_cholesky,  dense_qr, 
                                                                                                            // dense_cholesky and conjugate_gradients");
		options.preconditioner_type             =	ceres::JACOBI;                                          // DEFINE_string(preconditioner_type, "jacobi", "Options are:  identity, 
                                                                                                            // jacobi, schur_jacobi, cluster_jacobi,  cluster_tridiagonal");
		options.sparse_linear_algebra_library   =	 ceres::SUITE_SPARSE;                                   // DEFINE_string(sparse_linear_algebra_library, "suitesparse", "Options 
                                                                                                            // are: suitesparse and cxsparse");
		options.ordering_type                   =	ceres::SCHUR;                                           // DEFINE_string(ordering_type, "schur", "Options are: schur, user, natural");
		options.dogleg_type                     =	ceres::TRADITIONAL_DOGLEG;                              // DEFINE_string(dogleg_type, "traditional", "Options are: traditional, subspace");
		options.use_block_amd                   =	true;                                                   // DEFINE_bool(use_block_amd, true, "Use a block oriented fill reducing ordering.");
		options.num_threads                     =	1;                                                      // DEFINE_int32(num_threads, 1, "Number of threads");
		options.linear_solver_min_num_iterations=	5;                                                      // DEFINE_int32(num_iterations, 5, "Number of iterations");
		options.use_nonmonotonic_steps          =	false;                                                  // DEFINE_bool(nonmonotonic_steps, false, "Trust region 
                                                                                                            // algorithm can use nonmonotic steps");
		DEFINE_double( rotation_sigma, 0.0, "Standard deviation of camera rotation perturbation." );
		DEFINE_double( translation_sigma, 0.0, "Standard deviation of the camera translation perturbation." );
		DEFINE_double( point_sigma, 0.0, "Standard deviation of the point perturbation" );
		DEFINE_int32( random_seed, 38401, "Random seed used to set the state of the pseudo random number generator used to generate the pertubations." );
	*/  

	ceres::Solver::Summary summary;
	ceres::Solve( options, &problem, &summary );
	cout << summary.BriefReport() << endl;

	// obtain camera matrix from parameters
	for( int cam_id = 0; cam_id < n_cameras; cam_id++ )
    {
		double* camera_ptr = camera_parameter + 6 * cam_id;
		double* camera_mat = camera_matrix + 12 * cam_id;
		if( !( std::isnan( *camera_ptr ) ) )
        {
			ceres::AngleAxisToRotationMatrix< double >( camera_ptr, camera_mat );
			camera_mat[9]  = camera_ptr[3];
			camera_mat[10] = camera_ptr[4];
			camera_mat[11] = camera_ptr[5];
		}
	}

	for( int object_id = 0; object_id < n_objects; object_id )
    {
		double* object_ptr = object_parameter + 6 * object_id;
		double* object_mat = object_rot_trans + 12 * object_id;
		if( !( std::isnan( *object_ptr ) ) )
        {
			if( object_type[ object_id ] == 1 )
            {
				cout << "Type 1 object=" << object_id <<" : " << object_ptr[0] << " " << object_ptr[1] << " " << object_ptr[2] << endl;
				object_ptr[1] = -object_ptr[2];
				object_ptr[2] = 0;
			}

			ceres::AngleAxisToRotationMatrix<double>(object_ptr, object_mat);
			object_mat[9]  = object_ptr[3];
			object_mat[10] = object_ptr[4];
			object_mat[11] = object_ptr[5];
		}
	}


	// write back result files
	//  std::cout<<nCam<<" "<<nPts<<" "<<c<<std::endl;
	FILE* fpout = fopen( argv[4], "wb" );
	fwrite( (void*) (&n_cameras), sizeof(unsigned int), 1, fpout );
	fwrite( (void*) (&n_points), sizeof(unsigned int), 1, fpout );
	fwrite( (void*) (&n_objects), sizeof(unsigned int), 1, fpout );
	fwrite( (void*) (camera_matrix), sizeof(double), 12 * n_cameras, fpout );
	fwrite( (void*) (point_cloud), sizeof(double), 3 * n_points, fpout );
	fwrite( (void*) (object_rot_trans), sizeof(double), 12 * n_objects, fpout );
    fwrite( (void*) (box_constraints_var), sizeof(double), 3 * n_rand_pairs, fpout );



	fclose (fpout);

	//clean up
	delete [] camera_matrix;
	delete [] object_rot_trans;
	delete [] point_cloud;
	delete [] point_observed_index;
	delete [] point_observed_value;
	delete [] camera_parameter;
	delete [] object_parameter;
	delete [] object_half_size;
	delete [] object_weight;
	delete [] object_type;

	return 0;  
}
