#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres_functions.h"

using namespace std;

void generate_random_number_array( unsigned int *random_array, int n_rand_points, int max_pts )
{
	srand( (unsigned) time(0) ); 
	for( int idx = 0; idx < n_rand_points ; idx++ )
		random_array[ idx ]	=	( rand() % max_pts ) + 1;
 
	return;
}

bool check_repetition( unsigned int *added_pairs, int n_rand_pairs, int id_obs_one, int id_obs_two )
{
    for( int i = 0; i < n_rand_pairs; i++ )
        if( (added_pairs[2*i] == id_obs_one) && (added_pairs[2*i+1] == id_obs_two) ) return true;
        else if( (added_pairs[2*i] == id_obs_two) && (added_pairs[2*i+1] == id_obs_one ) ) return true;

    return false;
}


void ceres_fill_camera_parameters( double *camera_matrix, double *camera_parameter, int n_cameras )
{
	for( int camera_id = 0; camera_id < n_cameras; camera_id++ )
	{
		double *camera_ptr	=	camera_parameter + 6 * camera_id;
		double *camera_mat	=	camera_matrix + 12 * camera_id;
		if( !isnan( *camera_ptr ) )
		{
			ceres::RotationMatrixToAngleAxis< double >( camera_mat, camera_ptr );
			camera_ptr[3]	=	camera_mat[9];
			camera_ptr[4]	=	camera_mat[10];
			camera_ptr[5]	=	camera_mat[11];
		}

	}
	return;

	/*
	for(int cameraID=0; cameraID<nCam; ++cameraID){
		double* cameraPtr = camera_parameter+6*cameraID;
		double* cameraMat = camera_matrix+12*cameraID;
		if (!(std::isnan(*cameraPtr))){
			ceres::RotationMatrixToAngleAxis<double>(cameraMat, cameraPtr);
			cameraPtr[3] = cameraMat[9];
			cameraPtr[4] = cameraMat[10];
			cameraPtr[5] = cameraMat[11];
		}
	}
	*/
}

void ceres_fill_object_parameters( double *object_matrix, double *object_parameter, unsigned int *object_type, int n_objects )
{
	for( int object_id = 0; object_id < n_objects; object_id++ )
	{
		double *object_ptr	=	object_parameter + 6 * object_id;
		double *object_mat	=	object_matrix + 12 * object_id;
		if( !isnan( *object_ptr ) )
		{
			ceres::RotationMatrixToAngleAxis< double >( object_mat, object_ptr );
			if( object_type[ object_id ] == 1 ) object_ptr[2]	=	-object_ptr[1];
			object_ptr[3]	=	object_mat[9];
			object_ptr[4]	=	object_mat[10];
			object_ptr[5]	=	object_mat[11];
		}
	}
	return;

	/*
	for(int objectID=0; objectID<nObjects; ++objectID){
		double* objectPtr = object_parameter+6*objectID;
		double* objectMat = objectRt+12*objectID;
		if (!(std::isnan(*objectPtr))){
			ceres::RotationMatrixToAngleAxis<double>(objectMat, objectPtr);

			if (object_type[objectID]==1){
				//   std::cout<<"Type 1 object="<<objectID<<" : "<<objectPtr[0]<<" "<<objectPtr[1]<<" "<<objectPtr[2]<<std::endl;
				objectPtr[2] = -objectPtr[1];
			}

			//std::cout<<"Object "<<objectID<<" type= "<<object_type[objectID]<<" weight = "<<object_weight[objectID]<<std::endl;
			//std::cout<<"halfSize = "<<object_half_size[3*objectID]<<" "<<object_half_size[3*objectID+1]<< " " <<object_half_size[3*objectID+2]<<std::endl;

			objectPtr[3] = objectMat[9];
			objectPtr[4] = objectMat[10];
			objectPtr[5] = objectMat[11];
			//std::cout<<"cameraID="<<cameraID<<" : ";
			//std::cout<<"cameraPtr="<<cameraPtr[0]<<" "<<cameraPtr[1]<<" "<<cameraPtr[2]<<" "<<cameraPtr[3]<<" "<<cameraPtr[4]<<" "<<cameraPtr[5]<<std::endl;
		}
	}
	*/
}

void ceres_add_triangulation_function( double *obs_ptr, double *camera_ptr, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction< AlignmentErrorTriangulate, 2, 3 > ( new AlignmentErrorTriangulate( obs_ptr,camera_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, point_ptr );
	return;
}

void ceres_add_alignment_error_2d_function( double *obs_ptr, double *camera_ptr, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentError2D, 2, 6, 3 > ( new AlignmentError2D( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
	return;
}

void ceres_add_alignment_error_3d_function( double *obs_ptr, double *camera_ptr, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentError3D, 3, 6, 3 > ( new AlignmentError3D( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
	return;
}

void ceres_add_alignment_error_2d3d_function( double *obs_ptr, double *camera_ptr, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	if( isnan( obs_ptr[2] ) ) ceres_add_alignment_error_2d_function( obs_ptr, camera_ptr, point_ptr, loss_function, problem );
	else
	{
		ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentError2D3D, 5, 6, 3 > ( new AlignmentError2D3D( obs_ptr ) );
		problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
	}
	return;
}

void ceres_add_translation_error_function( double *obs_ptr, double *camera_ptr, double *cam_ptr_one, double *cam_ptr_two, 
										   ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentErrortran, 3, 6, 6, 6 > ( new AlignmentErrortran( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, cam_ptr_one, cam_ptr_two );
	return;
}

void ceres_add_translation_error1_function( double *obs_ptr, double *camera_ptr, double *cam_ptr_one, double *cam_ptr_two, 
										   ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentErrortran1, 3, 6, 6, 6 > ( new AlignmentErrortran1( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, cam_ptr_one, cam_ptr_two );
	return;
}


void ceres_add_alignment_traj_normal_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
													 double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame)
{
	ceres::CostFunction *cost_function		=	new ceres::AutoDiffCostFunction < AlignmentError2D3D1, 6, 6, 3 > ( new AlignmentError2D3D1( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
	if( not_first_frame ) ceres_add_translation_error_function( obs_ptr, camera_ptr, cam_ptr_one, cam_ptr_two, loss_function, problem );
	return;
}

void ceres_add_alignment_traj1_normal_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
													 double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame)
{
	ceres::CostFunction *cost_function		=	new ceres::AutoDiffCostFunction < AlignmentError2D3D1, 6, 6, 3 > ( new AlignmentError2D3D1( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
	if( not_first_frame ) ceres_add_translation_error1_function( obs_ptr, camera_ptr, cam_ptr_one, cam_ptr_two, loss_function, problem );
	return;
}

void ceres_add_alignment_traj_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
											  double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame )
{
	ceres::CostFunction *cost_function		=	new ceres::AutoDiffCostFunction < AlignmentError2D3D, 5, 6, 3 > ( new AlignmentError2D3D( obs_ptr ) );
	problem.AddResidualBlock( cost_function,loss_function, camera_ptr, point_ptr );
	if( not_first_frame ) ceres_add_translation_error_function( obs_ptr, camera_ptr, cam_ptr_one, cam_ptr_two, loss_function, problem );
	return;
}

void ceres_add_alignment_traj1_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
											  double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame )
{
	ceres::CostFunction *cost_function		=	new ceres::AutoDiffCostFunction < AlignmentError2D3D, 5, 6, 3 > ( new AlignmentError2D3D( obs_ptr ) );
	problem.AddResidualBlock( cost_function,loss_function, camera_ptr, point_ptr );
	if( not_first_frame ) ceres_add_translation_error1_function( obs_ptr, camera_ptr, cam_ptr_one, cam_ptr_two, loss_function, problem );
	return;
}

unsigned int ceres_add_alignment_traj_normal_box_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_cloud,
														 double *cam_ptr_one, double *cam_ptr_two, unsigned int *point_observed_index, 
														 ceres::LossFunction *loss_function, ceres::Problem &problem, 
														 unsigned int *random_array, double *box_constraints_var, int n_points, int n_cameras, 
														 int n_rand_pairs, int id_obs, int mode, int not_first_frame )
{
	double *point_ptr_two;
    unsigned int num_selected_pts       =   0;
    unsigned int *second_random_array   =   new unsigned int [ n_rand_pairs ];
    unsigned int *added_pairs           =   new unsigned int [ 2*n_rand_pairs ];

	ceres::CostFunction *cost_function		=	new ceres::AutoDiffCostFunction< AlignmentError2D3D, 5, 6, 3 > ( new AlignmentError2D3D( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr_one );

	// generate_random_number_array( random_array, n_rand_pairs, n_points );
	// generate_random_number_array( second_random_array, n_rand_pairs, n_points );
	if( not_first_frame )
	{
		int cntr = 0;

		for( unsigned int views = 0; views < n_cameras-1; views++ )
		{
            // if( views % 2 ) generate_random_number_array( second_random_array, n_rand_pairs, n_points );
            // else generate_random_number_array( random_array, n_rand_pairs, n_points );
			for( int first = 0; first < n_rand_pairs; first++ )
			{
				int id_obs_one = random_array[ first ];
				//if( id_obs_one == id_obs )
				{
					// for( int second = first+1; second < n_rand_pairs; second++ )
					{
						int id_obs_two	=	random_array[ first+n_rand_pairs ];
                        if( id_obs_one == id_obs_two ) continue;
                        if( check_repetition( added_pairs, n_rand_pairs, id_obs_one, id_obs_two ) ) continue ;

						point_ptr_two	=	point_cloud + point_observed_index[ 2*id_obs_two+1 ] * 3;
						if( point_ptr_one != point_ptr_two )
						{ 
							if( mode == 7 )
								ceres_add_box_constraints_all( obs_ptr, camera_ptr, point_ptr_one, point_ptr_two, box_constraints_var + 3*cntr, loss_function, problem );
							else if( mode == 8 )
								ceres_add_box_constraints_partial( obs_ptr, camera_ptr, point_ptr_one, point_ptr_two, box_constraints_var + cntr, loss_function, problem );
							else if( mode == 9 )
								ceres_add_box_constraints_vector( obs_ptr, camera_ptr, point_ptr_one, point_ptr_two, box_constraints_var, loss_function, problem );
							else
								ceres_add_box_constraints_scalar( obs_ptr, camera_ptr, point_ptr_one, point_ptr_two, box_constraints_var, loss_function, problem );

							if( mode < 9 ) cntr++;
                            added_pairs[ 2*num_selected_pts ]       =   id_obs_one;
                            added_pairs[ 2*num_selected_pts + 1 ]   =   id_obs_two;
                            num_selected_pts++;
                            if( num_selected_pts == n_rand_pairs ) break;
						}
					}
				}
                if( num_selected_pts == n_rand_pairs ) break;
			}
		}
	}
    delete[]    added_pairs;
    delete[]    second_random_array;
	return      num_selected_pts;
}

void ceres_add_box_constraints_vector( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_ptr_two, double* box_constraints_var, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorbox_new1,3,3,3,3,6>( new AlignmentErrorbox_new1( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, box_constraints_var, point_ptr_one, point_ptr_two, camera_ptr );
	problem.SetParameterLowerBound( box_constraints_var, 0, -3 );								// FREE VARIABLE WARNING
	problem.SetParameterUpperBound( box_constraints_var, 0, 3 );								// FREE VARIABLE WARNING
	return;
}

void ceres_add_box_constraints_scalar( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_ptr_two, double* box_constraints_var, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorbox_new,3,1,3,3,6>( new AlignmentErrorbox_new( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, box_constraints_var, point_ptr_one, point_ptr_two, camera_ptr );
	problem.SetParameterLowerBound( box_constraints_var, 0, -3 );								// FREE VARIABLE WARNING
	problem.SetParameterUpperBound( box_constraints_var, 0, 3 );								// FREE VARIABLE WARNING
	return;
}

void ceres_add_box_constraints_partial( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_ptr_two, double* box_constraints_var, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorbox_new,3,1,3,3,6>( new AlignmentErrorbox_new( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, box_constraints_var, point_ptr_one, point_ptr_two, camera_ptr );
	problem.SetParameterLowerBound( box_constraints_var, 0, -3 );								// FREE VARIABLE WARNING
	problem.SetParameterUpperBound( box_constraints_var, 0, 3 );								// FREE VARIABLE WARNING
	return;
}

void ceres_add_box_constraints_all( double *obs_ptr, double *camera_ptr, double *point_ptr_one, double *point_ptr_two, double* box_constraints_var, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorbox_new1,3,3,3,3,6>( new AlignmentErrorbox_new1( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, box_constraints_var, point_ptr_one, point_ptr_two, camera_ptr );
	problem.SetParameterLowerBound( box_constraints_var, 0, -3 );								// FREE VARIABLE WARNING
	problem.SetParameterUpperBound( box_constraints_var, 0, 3 );								// FREE VARIABLE WARNING
	return;
}


void ceres_add_alignment_error_2d_normal_function( double *obs_ptr, double *camera_ptr,double *cam_ptr_one, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentError2D, 2, 6, 3 > ( new AlignmentError2D( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
	if( isnan( obs_ptr[2] ) ) ceres_add_alignment_error_2d_function( obs_ptr, camera_ptr, point_ptr, loss_function, problem );
	else
	{
    cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorNormal,1, 6,6, 3>(new AlignmentErrorNormal(obs_ptr));
    problem.AddResidualBlock(cost_function,loss_function,camera_ptr,cam_ptr_one,point_ptr);
	}
	return;
}

void ceres_add_alignment_error_3d_normal_function( double *obs_ptr, double *camera_ptr,double *cam_ptr_one, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentError3D, 3, 6, 3 > ( new AlignmentError3D( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
	if( isnan( obs_ptr[2] ) ) ceres_add_alignment_error_2d_function( obs_ptr, camera_ptr, point_ptr, loss_function, problem );
	else
	{
    cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorNormal,1, 6,6, 3>(new AlignmentErrorNormal(obs_ptr));
    problem.AddResidualBlock(cost_function,loss_function,camera_ptr,cam_ptr_one,point_ptr);
	}
	return;
}

void ceres_add_alignment_error_2d3d_normal_function( double *obs_ptr, double *camera_ptr,double *cam_ptr_one, double *point_ptr, ceres::LossFunction *loss_function, ceres::Problem &problem )
{
	if( isnan( obs_ptr[2] ) ) ceres_add_alignment_error_2d_function( obs_ptr, camera_ptr, point_ptr, loss_function, problem );
	else
	{
		ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentError2D3D, 5, 6, 3 > ( new AlignmentError2D3D( obs_ptr ) );
		problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
		cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorNormal,1, 6,6, 3>(new AlignmentErrorNormal(obs_ptr));
        problem.AddResidualBlock(cost_function,loss_function,camera_ptr,cam_ptr_one,point_ptr);
	}
	return;
}


void ceres_add_alignment_2d_traj_normal_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
													 double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame )
{
	ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentError2D, 2, 6, 3 > ( new AlignmentError2D( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
    if( isnan( obs_ptr[2] ) ) ceres_add_alignment_error_2d_function( obs_ptr, camera_ptr, point_ptr, loss_function, problem );
	else
	{
    cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorNormal,1, 6,6, 3>(new AlignmentErrorNormal(obs_ptr));
    problem.AddResidualBlock(cost_function,loss_function,camera_ptr,cam_ptr_one,point_ptr);
	}
	if( not_first_frame ) ceres_add_translation_error_function( obs_ptr, camera_ptr, cam_ptr_one, cam_ptr_two, loss_function, problem );
	return;
}

void ceres_add_alignment_3d_traj_normal_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
													 double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame )
{
		ceres::CostFunction *cost_function	=	new ceres::AutoDiffCostFunction < AlignmentError3D, 3, 6, 3 > ( new AlignmentError3D( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
	if( isnan( obs_ptr[2] ) ) ceres_add_alignment_error_2d_function( obs_ptr, camera_ptr, point_ptr, loss_function, problem );
	else
	{
    cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorNormal,1, 6,6, 3>(new AlignmentErrorNormal(obs_ptr));
    problem.AddResidualBlock(cost_function,loss_function,camera_ptr,cam_ptr_one,point_ptr);
	}
	if( not_first_frame ) ceres_add_translation_error_function( obs_ptr, camera_ptr, cam_ptr_one, cam_ptr_two, loss_function, problem );
	return;
}


void ceres_add_alignment_2d3d_traj_normal_error_function( double *obs_ptr, double *camera_ptr, double *point_ptr, double *cam_ptr_one, 
													 double *cam_ptr_two, ceres::LossFunction *loss_function, ceres::Problem &problem, int not_first_frame )
{
	ceres::CostFunction *cost_function		=	new ceres::AutoDiffCostFunction < AlignmentError2D3D, 6, 6, 3 > ( new AlignmentError2D3D( obs_ptr ) );
	problem.AddResidualBlock( cost_function, loss_function, camera_ptr, point_ptr );
	if( isnan( obs_ptr[2] ) ) ceres_add_alignment_error_2d_function( obs_ptr, camera_ptr, point_ptr, loss_function, problem );
	else
	{
    cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorNormal,1, 6,6, 3>(new AlignmentErrorNormal(obs_ptr));
    problem.AddResidualBlock(cost_function,loss_function,camera_ptr,cam_ptr_one,point_ptr);
	}
	if( not_first_frame ) ceres_add_translation_error_function( obs_ptr, camera_ptr, cam_ptr_one, cam_ptr_two, loss_function, problem );
	return;
}







