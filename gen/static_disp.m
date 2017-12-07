function static_disp(str,varargin)
% based on code from the following url
% http://undocumentedmatlab.com/blog/command-window-text-manipulation/

% \33[2K look into this
%%
    persistent reverseStr
    
    if isempty(reverseStr)
        reverseStr = '';
    end

    msg = sprintf(str,varargin{:});
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));

end
