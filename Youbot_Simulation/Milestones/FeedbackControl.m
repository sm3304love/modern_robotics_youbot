function V_b = FeedbackControl(X, Xd, Xd_next, Kp, Ki, timestep)

    %[V_d] fomular from Milestone3, diffrent textbook fomular
    V_d = se3ToVec(MatrixLog6(TransInv(Xd)*Xd_next)/timestep);
    
    %MatrixLog6 -> log(Matrix)
    X_e = se3ToVec( MatrixLog6(TransInv(X)*Xd) );
    
    Ad_xinv_xd = Adjoint(TransInv(X)*Xd);
    
    %integral?
    Integral_X_e = X_e+ X_e*timestep;

    V_b = Ad_xinv_xd*V_d+Kp*X_e+Ki*Integral_X_e;
end