function detectionObjects = clusterDetectionsY1(detections, vehicleSize)
    N = numel(detections);
    distances = zeros(N);
    for i = 1:N
        for j = i+1:N
            distances(i,j) = norm(detections(i).Measurement(1:2) - ...
                detections(j).Measurement(1:2));
        end
    end
    leftToCheck = 1:N;
    i = 0;
    detectionObjects = objectDetection(N,1);
    while ~isempty(leftToCheck)
        % Remove the detections that are in the same cluster as the one under
        % consideration
        underConsideration = leftToCheck(1);
        clusterInds = (distances(underConsideration, leftToCheck) < vehicleSize);
        detInds = leftToCheck(clusterInds);
        clusterDets = [detections(detInds)];
        clusterMeas = reshape([clusterDets.Measurement],[],2);
        meas = mean(clusterMeas, 1);
        meas2D = meas';
        i = i + 1;
        detectionObjects(i) = detections(detInds(1));
        detectionObjects(i).Measurement = meas2D;
        leftToCheck(clusterInds) = [];
    end
    detectionObjects(i+1:end) = [];

%     % Since the detections are now for clusters, modify the noise to 
%     % represent that they are of the whole car
%     for i = 1:numel(detectionObjects)
%         measNoise(1:2,1:2) = vehicleSize^2 * eye(2);
%         measNoise(3:4,3:4) = eye(2) * 100 * vehicleSize^2;
%         detectionObjects(i).MeasurementNoise = measNoise;
%     end
end