function text_out = formatMixedFontText(text_in, english_font_name, chinese_font_name)
% FORMATMIXEDFONTTEXT 为 tex 解释器字符串插入中英文字体切换

if nargin < 2 || isempty(english_font_name)
    if isappdata(groot, 'DefaultEnglishFontName')
        english_font_name = getappdata(groot, 'DefaultEnglishFontName');
    else
        english_font_name = 'Times New Roman';
    end
end

if nargin < 3 || isempty(chinese_font_name)
    if isappdata(groot, 'DefaultChineseFontName')
        chinese_font_name = getappdata(groot, 'DefaultChineseFontName');
    else
        chinese_font_name = 'SimSun';
    end
end

if ischar(text_in)
    text_out = format_one_text(text_in, english_font_name, chinese_font_name);
    return;
end

if isstring(text_in)
    if isscalar(text_in)
        text_out = format_one_text(char(text_in), english_font_name, chinese_font_name);
    else
        text_out = cell(size(text_in));
        for i = 1:numel(text_in)
            text_out{i} = format_one_text(char(text_in(i)), english_font_name, chinese_font_name);
        end
    end
    return;
end

if iscell(text_in)
    text_out = format_cell_text(text_in, english_font_name, chinese_font_name);
    return;
end

error('不支持的文本输入类型。');

end

function text_out = format_one_text(text_in, english_font_name, chinese_font_name)

text_out = ['\fontname{', english_font_name, '}'];
use_chinese_font = false;
for i = 1:length(text_in)
    ch = text_in(i);

    if is_chinese_char(ch)
        if ~use_chinese_font
            text_out = [text_out, '\fontname{', chinese_font_name, '}']; %#ok<AGROW>
            use_chinese_font = true;
        end
    else
        if use_chinese_font
            text_out = [text_out, '\fontname{', english_font_name, '}']; %#ok<AGROW>
            use_chinese_font = false;
        end
    end

    text_out = [text_out, ch]; %#ok<AGROW>
end

if use_chinese_font
    text_out = [text_out, '\fontname{', english_font_name, '}'];
end

end

function text_out = format_cell_text(text_in, english_font_name, chinese_font_name)

text_out = cell(size(text_in));

for i = 1:numel(text_in)
    item = text_in{i};

    if ischar(item)
        text_out{i} = format_one_text(item, english_font_name, chinese_font_name);
    elseif isstring(item) && isscalar(item)
        text_out{i} = format_one_text(char(item), english_font_name, chinese_font_name);
    elseif iscell(item)
        text_out{i} = format_cell_text(item, english_font_name, chinese_font_name);
    else
        error('单元格中的文本元素类型不受支持。');
    end
end

end

function tf = is_chinese_char(ch)

code = double(ch);

tf = (code >= 13312 && code <= 40959) || ...
    (code >= 12288 && code <= 12351) || ...
    (code >= 65281 && code <= 65374);

end
