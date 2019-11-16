{% for item in command_variables %}
{{ msg_type }} got_{{ item }}
{%- endfor %}
{% for item in measured_variables %}
{{ msg_type }} {{ item }}
{%- endfor %}