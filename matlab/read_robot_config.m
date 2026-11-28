function C = read_robot_config(fname)
%READ_ROBOT_CONFIG  把 robot_config.h 里的数值型 #define 解析成 struct
%   字段名 = 宏名，值 = 数值（去掉 f/U 后缀）。只解析数字字面量；
%   带表达式的宏（如 PID_MIDDLE_TICK_DIV）和 0x.. 会被跳过。
%
%   用法：C = read_robot_config();  C.PID_ATT_KP_PITCH 等
if nargin < 1
    here = fileparts(mfilename('fullpath'));
    fname = fullfile(here, '..', 'prj', 'elet2026', 'tasks', 'robot_config.h');
end
txt = fileread(fname);
lines = regexp(txt, '\r?\n', 'split');
C = struct();
pat = '^\s*#define\s+([A-Za-z_]\w*)\s+([-+]?\d+\.?\d*)([fFUu]?)\s*(?:/\*.*|//.*)?$';
for i = 1:numel(lines)
    tok = regexp(lines{i}, pat, 'tokens', 'once');
    if ~isempty(tok)
        C.(tok{1}) = str2double(tok{2});
    end
end
end
