function mu = manipulability(J,measure)
    % Input: J: a 6 × 6 matrix
    % Input: measure: a single string argument that can only be one of 
    % 'sigmamin', 'detjac', or 'invcond’. Defines which manipulability 
    % measure is used.
    % Output: Output: mu: The corresponding measure of manipulability
    sigmas = svd(J);
    switch measure
        case 'sigmamin'
            mu = min(sigmas);
        case 'detjac'
            mu = det(J);
        case 'invcond'
            mu = min(sigmas)/max(sigmas);
        otherwise
            error('invalid measure type');
    end
end