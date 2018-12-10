% THis program update the trace data
function [vel_update,vel_trace,acc_trace,rmse_trace,dist_err_trace,angle_err_trace,ref_ID_hist]=update_data_trace(xy_vel_acc,vel_trace,acc_trace,reflector_rmse,rmse_trace,matched_ref_ID_hist,ref_ID_hist,dist_err,dist_err_trace,angle_err,angle_err_trace)

            vel_update = [xy_vel_acc(1,1) xy_vel_acc(1,2)];
            vel_trace=[vel_trace; vel_update];
            acc_update = [xy_vel_acc(2,1) xy_vel_acc(2,2)];
            acc_trace=[acc_trace; acc_update];
            rmse_trace=[rmse_trace; reflector_rmse];
            [l_ref,w_ref]=size(matched_ref_ID_hist);
            if length(dist_err)<w_ref || length(angle_err)<w_ref
                dist_err=dist_err_trace(end,:)
                angle_err=angle_err_trace(end,:)
            end
            dist_err_trace=[dist_err_trace;dist_err];
            angle_err_trace=[angle_err_trace;angle_err];
            %pose_hist=[pose_hist;rotation_trace];
            if l_ref>w_ref
                matched_ref_ID_hist=matched_ref_ID_hist';
            end
            ref_ID_hist=[ref_ID_hist; matched_ref_ID_hist];